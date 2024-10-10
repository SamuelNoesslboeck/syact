use syunit::*;

// State
pub trait AsyncActuatorState {
    fn direction(&self) -> Direction;

    fn speed_factor(&self) -> Factor;
}

// Error type
pub enum AsyncActuatorError {
 
}

/// A component which is asynchronous because of its hardware properties, e.g. a simple DC-Motors
pub trait AsyncActuator {
    /// Starts the movement process of the component in the given direction with a given speed factor
    /// 
    /// # Panics
    /// 
    /// Panics if the percentage given is greater than 1.0 or less than 0.0
    fn drive_factor(&mut self, dir : Direction, speed : Factor) -> Result<(), AsyncActuatorError>; 

    fn drive_speed(&mut self, dir : Direction, speed : Velocity) -> Result<(), AsyncActuatorError>;

    // Data
        
    //
}