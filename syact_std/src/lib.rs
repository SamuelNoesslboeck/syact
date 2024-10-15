#![crate_name = "syact_std"]

use embedded_hal::digital::{OutputPin, PinState};
use syact::act::stepper::{StepperController, StepperControllerError};
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

    fn set_dir(&mut self, dir : Direction) -> Result<(), StepperControllerError> {
        self.pin_dir.set_state(PinState::from(dir.as_bool())).map_err(|_| StepperControllerError::IOError)
    }

    fn step(&mut self, time : Time) -> Result<(), StepperControllerError> {
        todo!()
    }
}