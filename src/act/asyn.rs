use syunit::*;

use crate::Setup;

/// A component which is asynchronous because of its hardware properties, e.g. a simple DC-Motors
pub trait AsyncActuator : Setup {
    /// Starts the movement process of the component in the given direction with a given speed factor
    /// 
    /// # Panics
    /// 
    /// Panics if the percentage given is greater than 1.0 or less than 0.0
    fn drive(&mut self, dir : Direction, speed : Factor) -> Result<(), crate::Error>; 

    // Data
        /// Returns the current movement direction
        fn dir(&self) -> Direction;
        
        /// Returns the current speed factor
        fn speed(&self) -> Factor;

        // fn vars<'a>(&'a self) -> &'a CompVars;
    //
}