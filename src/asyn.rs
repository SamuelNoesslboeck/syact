use syunit::*;

use crate::ActuatorError;

/// A component which is asynchronous because of its hardware properties, e.g. a simple DC-Motors
pub trait AsyncActuator<U : UnitSet> {
    /// Starts the movement process of the component in the given direction with a given `speed` factor
    fn drive_factor(&mut self, speed : Factor, direction : Direction) -> Result<(), ActuatorError<U>>; 

    /// Start the movement process of the component with the given velocity `speed`, positive values for `speed` mean CW movement
    fn drive_speed(&mut self, speed : U::Velocity) -> Result<(), ActuatorError<U>>;
}