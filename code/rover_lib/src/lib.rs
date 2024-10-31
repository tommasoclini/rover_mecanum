#![no_std]

use embedded_hal_1::{
    digital::{OutputPin, PinState},
    pwm::SetDutyCycle,
};
// use uom::si::f32::Angle;

pub enum Direction {
    Clockwise,
    CounterClockwise,
}

pub trait Motor {
    type Error;

    fn drive(&mut self, power: u8, dir: Direction) -> Result<(), Self::Error>;
    fn neutral(&mut self) -> Result<(), Self::Error>;
}

pub struct MyMotor<P: SetDutyCycle, O0: OutputPin, O1: OutputPin> {
    pwm: P,
    dir_0: O0,
    dir_1: O1,
    dir_active: PinState,
    dir_passive: PinState,
}

impl<P: SetDutyCycle, O0: OutputPin, O1: OutputPin> MyMotor<P, O0, O1> {
    pub fn new(pwm: P, dir_0: O0, dir_1: O1, dir_active: PinState) -> Self {
        Self {
            pwm,
            dir_0,
            dir_1,
            dir_active,
            dir_passive: dir_active.opposite(),
        }
    }
}

trait Opposite {
    fn opposite(&self) -> Self;
}

impl Opposite for PinState {
    fn opposite(&self) -> Self {
        match self {
            Self::High => Self::Low,
            Self::Low => Self::High,
        }
    }
}

impl<P: SetDutyCycle, O0: OutputPin, O1: OutputPin> Motor for MyMotor<P, O0, O1> {
    type Error = ();

    fn drive(&mut self, power: u8, dir: Direction) -> Result<(), Self::Error> {
        let dirs = match dir {
            Direction::Clockwise => (self.dir_active, self.dir_passive),
            Direction::CounterClockwise => (self.dir_passive, self.dir_active),
        };

        self.dir_0.set_state(dirs.0).map_err(|_| ())?;
        self.dir_1.set_state(dirs.1).map_err(|_| ())?;
        self.pwm
            .set_duty_cycle_fraction(power as u16, u8::MAX as u16)
            .map_err(|_| ())?;

        Ok(())
    }
    fn neutral(&mut self) -> Result<(), ()> {
        self.pwm.set_duty_cycle_fully_off().map_err(|_| ())?;
        self.dir_0.set_state(self.dir_passive).map_err(|_| ())?;
        self.dir_1.set_state(self.dir_passive).map_err(|_| ())?;

        Ok(())
    }
}