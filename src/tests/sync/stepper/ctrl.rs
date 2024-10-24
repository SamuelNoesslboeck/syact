use syunit::*;

use crate::prelude::*;

/// A simulated controller that does nothing
pub struct SimulatedController {
    _dir : Direction
}

impl SimulatedController {
    /// Creates a new simulated controller
    pub fn new() -> Self {
        Self {
            _dir: Direction::default()
        }
    }
}

impl StepperController for SimulatedController {
    fn step(&mut self, time : Seconds) -> Result<(), ActuatorError> {
        spin_sleep::sleep(time.into());
        Ok(())
    }

    fn direction(&self) -> Direction {
        self._dir
    }

    fn set_dir(&mut self, dir : Direction) -> Result<(), ActuatorError> {
        self._dir = dir;
        Ok(())
    }
}