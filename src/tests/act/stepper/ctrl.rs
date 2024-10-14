use syunit::*;

use crate::act::stepper::{StepperController, StepperControllerError};

pub struct SimulatedController {
    _dir : Direction
}

impl SimulatedController {
    pub fn new() -> Self {
        Self {
            _dir: Direction::default()
        }
    }
}

impl StepperController for SimulatedController {
    fn step(&mut self, time : Time) -> Result<(), StepperControllerError> {
        spin_sleep::sleep(time.into());
        Ok(())
    }

    fn dir(&self) -> Direction {
        self._dir
    }

    fn set_dir(&mut self, dir : Direction) {
        self._dir = dir;
    }
}