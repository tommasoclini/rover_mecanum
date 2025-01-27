#![no_std]
#![no_main]

extern crate alloc;

use alloc::{rc::Rc, sync::Arc};
use cobs::CobsDecoder;
use defmt::{debug, warn, Debug2Format, Display2Format};
use embassy_futures::select::Either;
use embassy_sync::{
    blocking_mutex::raw::{self as raw_mutex, CriticalSectionRawMutex, NoopRawMutex},
    mutex::Mutex,
    signal,
};
use embedded_alloc::LlffHeap as Heap;
use serde::{Deserialize, Serialize};
use serde_json::Value;
use uom::si::angle;

#[global_allocator]
static HEAP: Heap = Heap::empty();

use core::cell::RefCell;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::{task, Spawner};
use embassy_stm32::{
    bind_interrupts,
    exti::{Channel, ExtiInput},
    gpio::{AnyPin, Input, Output, Pin},
    peripherals,
    timer::simple_pwm,
    usart::{self, BufferedUart},
};
use embassy_time::{Duration, Timer};
use embedded_hal_02::PwmPin;

use defmt::info;

use embedded_io_async::BufRead;

use rover_lib::{
    iface::{FWRMerror, MecanumPower},
    my_lib::MyFourWheelRobotError,
    Angle, MecanumRobot, MyFourWheelRobot, MyMotor, Turn,
};

struct PwmWrapper<C, T, D, P: embedded_hal_02::Pwm<Channel = C, Time = T, Duty = D>> {
    pwm: Rc<RefCell<P>>,
    channel: C,
}

impl<C, T, D, P> PwmWrapper<C, T, D, P>
where
    P: embedded_hal_02::Pwm<Channel = C, Time = T, Duty = D>,
{
    pub fn new(pwm: Rc<RefCell<P>>, channel: C) -> Self {
        Self { pwm, channel }
    }
}

impl<C: Copy, T, D, P: embedded_hal_02::Pwm<Channel = C, Time = T, Duty = D>> PwmPin
    for PwmWrapper<C, T, D, P>
{
    type Duty = D;

    fn disable(&mut self) {
        self.pwm.borrow_mut().disable(self.channel);
    }
    fn enable(&mut self) {
        self.pwm.borrow_mut().enable(self.channel);
    }

    fn get_duty(&self) -> Self::Duty {
        self.pwm.borrow_mut().get_duty(self.channel)
    }
    fn get_max_duty(&self) -> Self::Duty {
        self.pwm.borrow_mut().get_max_duty()
    }
    fn set_duty(&mut self, duty: Self::Duty) {
        self.pwm.borrow_mut().set_duty(self.channel, duty);
    }
}

impl<C, T, D, P: embedded_hal_02::Pwm<Channel = C, Time = T, Duty = D>>
    embedded_hal_1::pwm::ErrorType for PwmWrapper<C, T, D, P>
{
    type Error = embedded_hal_1::pwm::ErrorKind;
}
impl<C: Copy, T, D, P> embedded_hal_1::pwm::SetDutyCycle for PwmWrapper<C, T, D, P>
where
    D: TryFrom<u16> + Into<u16>,
    P: embedded_hal_02::Pwm<Channel = C, Time = T, Duty = D>,
{
    fn max_duty_cycle(&self) -> u16 {
        self.get_max_duty().into()
    }
    fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
        self.set_duty(duty.try_into().map_err(|_| Self::Error::Other)?);
        Ok(())
    }
}

#[embassy_executor::task]
async fn rover_task(
    button: ExtiInput<'static, AnyPin>,
    robot: Arc<
        Mutex<raw_mutex::NoopRawMutex, dyn MecanumRobot<Error = FWRMerror<MyFourWheelRobotError>>>,
    >,
) {
    generic_rover_task(button, robot).await;
}

async fn generic_rover_task<E: core::error::Error>(
    mut button: ExtiInput<'_, AnyPin>,
    robot: Arc<Mutex<raw_mutex::NoopRawMutex, dyn (MecanumRobot<Error = E>)>>,
) {
    loop {
        button.wait_for_low().await;
        info!("making robot go forward");
        robot
            .lock()
            .await
            .drive(
                MecanumPower::new(1.0),
                Angle::new::<angle::radian>(core::f32::consts::FRAC_PI_2),
                Turn::new(0.0),
            )
            .unwrap();

        button.wait_for_high().await;
        info!("putting robot in neutral");
        robot.lock().await.neutral().unwrap();
    }
}

bind_interrupts!(struct Irqs {
    USART6 => usart::BufferedInterruptHandler<peripherals::USART6>;
});

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct RxMessage {
    p: Option<MecanumPower>,
    th: Option<Angle>,
    tu: Option<Turn>,
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    // allocator
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 0x4000;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    }

    let pwm = {
        use embassy_stm32::{gpio::OutputType, time::khz, timer::Channel};
        use simple_pwm::PwmPin;

        let channels = (
            Some(PwmPin::new_ch1(p.PA8, OutputType::PushPull)),
            Some(PwmPin::new_ch2(p.PA9, OutputType::PushPull)),
            Some(PwmPin::new_ch3(p.PA10, OutputType::PushPull)),
            Some(PwmPin::new_ch4(p.PA11, OutputType::PushPull)),
        );

        let mut pwm = simple_pwm::SimplePwm::new(
            p.TIM1,
            channels.0,
            channels.1,
            channels.2,
            channels.3,
            khz(1),
            Default::default(),
        );

        pwm.enable(Channel::Ch1);
        pwm.enable(Channel::Ch2);
        pwm.enable(Channel::Ch3);
        pwm.enable(Channel::Ch4);

        Rc::new(RefCell::new(pwm))
    };

    let robot = {
        use embassy_stm32::{
            gpio::{Level, Speed},
            timer::Channel,
        };
        use embedded_hal_1::digital::PinState;

        if cfg!(feature = "old_circuit") {
            MyFourWheelRobot::new(
                MyMotor::new(
                    PwmWrapper::new(Rc::clone(&pwm), Channel::Ch1),
                    Output::new(p.PC4.degrade(), Level::Low, Speed::Low),
                    Output::new(p.PB13.degrade(), Level::Low, Speed::Low),
                    PinState::High,
                ),
                MyMotor::new(
                    PwmWrapper::new(Rc::clone(&pwm), Channel::Ch2),
                    Output::new(p.PB14.degrade(), Level::Low, Speed::Low),
                    Output::new(p.PB15.degrade(), Level::Low, Speed::Low),
                    PinState::High,
                ),
                MyMotor::new(
                    PwmWrapper::new(Rc::clone(&pwm), Channel::Ch3),
                    Output::new(p.PB1.degrade(), Level::Low, Speed::Low),
                    Output::new(p.PB2.degrade(), Level::Low, Speed::Low),
                    PinState::High,
                ),
                MyMotor::new(
                    PwmWrapper::new(Rc::clone(&pwm), Channel::Ch4),
                    Output::new(p.PB12.degrade(), Level::Low, Speed::Low),
                    Output::new(p.PC5.degrade(), Level::Low, Speed::Low),
                    PinState::High,
                ),
            )
        } else {
            MyFourWheelRobot::new(
                MyMotor::new(
                    PwmWrapper::new(Rc::clone(&pwm), Channel::Ch1),
                    Output::new(p.PC0.degrade(), Level::Low, Speed::Low),
                    Output::new(p.PC1.degrade(), Level::Low, Speed::Low),
                    PinState::High,
                ),
                MyMotor::new(
                    PwmWrapper::new(Rc::clone(&pwm), Channel::Ch2),
                    Output::new(p.PC2.degrade(), Level::Low, Speed::Low),
                    Output::new(p.PC3.degrade(), Level::Low, Speed::Low),
                    PinState::High,
                ),
                MyMotor::new(
                    PwmWrapper::new(Rc::clone(&pwm), Channel::Ch3),
                    Output::new(p.PC5.degrade(), Level::Low, Speed::Low),
                    Output::new(p.PC10.degrade(), Level::Low, Speed::Low),
                    PinState::High,
                ),
                MyMotor::new(
                    PwmWrapper::new(Rc::clone(&pwm), Channel::Ch4),
                    Output::new(p.PC11.degrade(), Level::Low, Speed::Low),
                    Output::new(p.PC12.degrade(), Level::Low, Speed::Low),
                    PinState::High,
                ),
            )
        }
    };

    let button: ExtiInput<'static, AnyPin> = ExtiInput::new(
        Input::new(p.PC13.degrade(), embassy_stm32::gpio::Pull::Up),
        p.EXTI13.degrade(),
    );
    let robot_m = Arc::new(Mutex::new(robot));

    static SIGNAL: signal::Signal<CriticalSectionRawMutex, ()> = const {signal::Signal::new()};

    spawner.spawn(rover_task(button, robot_m.clone())).unwrap();
    spawner.spawn(safety_timer(robot_m.clone(), &SIGNAL)).unwrap();

    const RX_SIZE: usize = 128;

    let mut tx_buf = [0u8; 32];
    let mut rx_buf = [0u8; RX_SIZE];

    let buf_usart = BufferedUart::new(
        p.USART6,
        Irqs,
        p.PC7,
        p.PC6,
        &mut tx_buf,
        &mut rx_buf,
        usart::Config::default(),
    )
    .unwrap();

    #[allow(unused)]
    let (mut tx, mut rx) = buf_usart.split();

    let mut p = MecanumPower::default();
    let mut th = Angle::default();
    let mut tu = Turn::default();

    loop {
        let mut decode_out = [0u8; RX_SIZE];

        let mut decoder = CobsDecoder::new(&mut decode_out);
        let size = loop {
            let buf = rx.fill_buf().await.unwrap();
            let len = buf.len();

            debug!(
                "received raw: {:?}",
                Debug2Format(&core::str::from_utf8(buf))
            );

            match decoder.push(buf) {
                Ok(Some((n, m))) => {
                    rx.consume(m);
                    break Some(n);
                }
                Ok(None) => {
                    rx.consume(len);
                }
                Err(_) => {
                    rx.consume(len);
                    warn!("error decoding cobs");
                    break None;
                }
            }
        };

        if let Some(size) = size {
            let packet_raw = &decode_out[..size];

            let Ok(rx_message) = serde_json::from_slice::<RxMessage>(packet_raw) else {
                continue;
            };
            SIGNAL.signal(());

            let mut change_needed = false;

            rx_message.p.inspect(|v| {
                p = *v;
                change_needed = true;
            });
            rx_message.th.inspect(|v| {
                th = *v;
                change_needed = true;
            });
            rx_message.tu.inspect(|v| {
                tu = *v;
                change_needed = true;
            });

            if change_needed {
                debug!(
                    "p: {}, th: {}, tu: {}",
                    p.inner(),
                    th.get::<uom::si::angle::radian>(),
                    tu.inner()
                );
                _ = robot_m
                    .lock()
                    .await
                    .drive(p, th, tu)
                    .inspect(|_| info!("all went well"))
                    .inspect_err(|_| warn!("failed to drive robot"));
            };
        }
    }
}

type SafetyMutex = CriticalSectionRawMutex;

#[task]
async fn safety_timer(
    robot: Arc<
        Mutex<NoopRawMutex, dyn MecanumRobot<Error = FWRMerror<MyFourWheelRobotError>>>,
    >,
    sig: &'static signal::Signal<SafetyMutex, ()>,
) {
    safety_timer_generic(robot, sig).await;
}

async fn safety_timer_generic<E: core::error::Error>(
    robot: Arc<Mutex<NoopRawMutex, dyn (MecanumRobot<Error = E>)>>,
    sig: &'static signal::Signal<SafetyMutex, ()>,
) {
    loop {
        let Either::First(_) =
            embassy_futures::select::select(async { Timer::after_millis(500).await }, async {
                sig.wait().await
            })
            .await
        else {
            continue;
        };
        robot
            .lock()
            .await
            .neutral()
            .expect("failed to stop robot in safety timer");
    }
}
