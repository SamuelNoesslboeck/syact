use alloc::sync::Arc;

use syunit::*;

use crate::act::ActuatorError;

// State
/// The state of an `AsyncActuator`
pub trait AsyncActuatorState {
    /// The current movement direction
    fn direction(&self) -> Direction;

    /// The current speed factor (percentage of max speed)
    fn speed_factor(&self) -> Factor;
}

/// A component which is asynchronous because of its hardware properties, e.g. a simple DC-Motors
pub trait AsyncActuator {
    /// Starts the movement process of the component in the given direction with a given `speed` factor
    fn drive_factor(&mut self, speed : Factor, direction : Direction) -> Result<(), ActuatorError>; 

    /// Start the movement process of the component with the given velocity `speed`, positive values for `speed` mean CW movement
    fn drive_speed(&mut self, speed : Velocity) -> Result<(), ActuatorError>;

    // Data
        /// Returns a reference to the `AsyncActuatorState` of the component
        fn state(&self) -> &dyn AsyncActuatorState;

        /// Clone the state of the `AsyncActuatorState` of the component
        fn clone_state(&self) -> Arc<dyn AsyncActuatorState>;
    //
}