use alloc::sync::Arc;

use syunit::*;

use crate::act::ActuatorError;

// State
pub trait AsyncActuatorState {
    fn direction(&self) -> Direction;

    fn speed_factor(&self) -> Factor;
}

/// A component which is asynchronous because of its hardware properties, e.g. a simple DC-Motors
pub trait AsyncActuator {
    /// Starts the movement process of the component in the given direction with a given speed factor
    fn drive_factor(&mut self, direction : Direction, speed : Factor) -> Result<(), ActuatorError>; 


    fn drive_speed(&mut self, direction : Direction, speed : Velocity) -> Result<(), ActuatorError>;

    // Data
        fn state(&self) -> &dyn AsyncActuatorState;

        fn clone_state(&self) -> Arc<dyn AsyncActuatorState>;
    //
}