#![doc = include_str!("../README.md")]
#![crate_name = "syact"]
#![deny(missing_docs)]
#![cfg_attr(not(any(test, feature = "testing")), no_std)]
#![cfg_attr(feature = "testing", allow(unused))]

// Modules
extern crate alloc;

// ####################
// #    SUBMODULES    #
// ####################
    // Core submodules
        /// Actuator structures and traits
        pub mod act;
        pub use act::{ActuatorError, AsyncActuator, AsyncActuatorState, SyncActuator, SyncActuatorGroup, SyncActuatorState, MiniServo, SyncActuatorBlocking};
    
        /// Structs for storing characteristics of stepper motors and devices
        pub mod data;
        pub use data::{MicroSteps, StepperConst, StepperConfig};

        /// Functions and Structs for calculating Stepper Motor procedures and operations
        pub mod math;

        /// Functions and Structs for taking measurements with a robot for e.g. position calculation
        pub mod meas;
    // 

    // Include proc_macro
    pub use syact_macros::{SyncActuatorGroup, StepperActuatorGroup};

    /// Easy import of the functionalities
    pub mod prelude;

    /// Module with all the tests required to assure the library funcitons as intended
    #[cfg(any(test, feature = "testing"))]
    mod tests;

    pub use syunit as units;
// 

// ###########################
// #    SETUP & DISMANTLE    #
// ###########################
    /// A trait that provides a universal setup function, 
    /// 
    /// # Pin management
    /// 
    /// For dynamic initialization purposes, all pin creations should run in a `setup()` function
    pub trait Setup {
        /// Error that can occur when setting up
        type Error;

        /// Calls all required functions to assure the components functionality
        fn setup(&mut self) -> Result<(), Self::Error> { 
            Ok(()) 
        }

        /// Points to `setup``, helper function
        fn setup_inline(mut self) -> Result<Self, Self::Error> 
        where 
            Self : Sized 
        {
            self.setup()?;
            Ok(self)
        }
    }

    /// A trait that provides a universal dismantle function
    /// 
    /// # Pin management
    /// 
    /// For dynamic initialization purposes, all pins that have previously been created with `setup()` should be dropped here
    pub trait Dismantle {
        /// Error that can occur when dismantling
        type Error;

        /// Calls all required functions to assure the component will not occupy any more resources like pins or network connections, without dropping the value
        fn dismantle(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }

        /// Calls `dismantle()` with an owned object
        fn dismantle_inline(mut self) -> Result<Self, Self::Error> 
        where
            Self : Sized
        {
            self.dismantle()?;
            Ok(self)
        }
    }
//