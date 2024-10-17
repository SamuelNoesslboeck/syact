use crate::ActuatorError;
use syunit::*;

/// A controller for the logics of a stepper motor
pub trait StepperController {
    /// Initializes a step with the given `time`, this function will set the pin to `HIGH` 
    fn step(&mut self, time : Time) -> Result<(), ActuatorError>;

    /// The movement direction of the motor
    fn direction(&self) -> Direction;

    /// Sets the direction of the motor
    fn set_dir(&mut self, dir : Direction) -> Result<(), ActuatorError>;
}