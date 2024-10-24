#![doc = include_str!("../README.md")]
#![crate_name = "syact_std"]

use embedded_hal::digital::{OutputPin, PinState};

use syact::ActuatorError;
use syact::sync::stepper::StepperController;
use syact::units::*;

pub struct GenericPWMController<DIR : OutputPin, STEP : OutputPin> {
    pin_dir : DIR,
    pin_step : STEP,

    direction : Direction,
}

impl<DIR : OutputPin, STEP : OutputPin> GenericPWMController<DIR, STEP> {
    pub fn new(pin_dir : DIR, pin_step : STEP) -> Self {
        Self {
            pin_dir,
            pin_step,

            direction: Direction::default()
        }
    }
}

impl<DIR : OutputPin, STEP : OutputPin> StepperController for GenericPWMController<DIR, STEP> {
    fn direction(&self) -> Direction {
        self.direction
    }

    fn set_dir(&mut self, dir : Direction) -> Result<(), ActuatorError> {
        self.pin_dir.set_state(PinState::from(dir.as_bool())).map_err(|_| ActuatorError::IOError)
    }

    fn step(&mut self, time : Seconds) -> Result<(), ActuatorError> {
        self.pin_step.set_high().map_err(|_| ActuatorError::IOError)?;
        spin_sleep::sleep((time / 2.0).into());
        self.pin_step.set_low().map_err(|_| ActuatorError::IOError)?;
        spin_sleep::sleep((time / 2.0).into());
        Ok(())
    }
}