use syunit::*;

use crate::act::ActuatorError;

/// A component which is asynchronous because of its hardware properties, e.g. a simple DC-Motors
pub trait AsyncActuator {
    /// Starts the movement process of the component in the given direction with a given `speed` factor
    fn drive_factor(&mut self, speed : Factor, direction : Direction) -> Result<(), ActuatorError>; 

    /// Start the movement process of the component with the given velocity `speed`, positive values for `speed` mean CW movement
    fn drive_speed(&mut self, speed : Velocity) -> Result<(), ActuatorError>;
}