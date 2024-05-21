#![doc = include_str!("../README.md")]
#![crate_name = "syact"]
#![deny(missing_docs)]

// Modules
extern crate alloc;

// ####################
// #    SUBMODULES    #
// ####################
        /// Actuator structures and traits
        pub mod act;
        pub use act::{SyncActuator, SyncActuatorGroup, Stepper};
        pub use act::asyn::AsyncActuator;

        /// Various devices
        pub mod device;
    
        /// Structs for storing characteristics of stepper motors and devices
        pub mod data;
        pub use data::{StepperConst, StepperConfig, ActuatorVars, MicroSteps};

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
// 

// ################
// #    ERRORS    #
// ################
// 
// Different platforms require different types of errors 
    // Wrapped types
    /// The general error type used in the crate
    pub type Error = Box<dyn std::error::Error>;
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
        /// Calls all required functions to assure the components functionality
        fn setup(&mut self) -> Result<(), Error> { 
            Ok(()) 
        }

        /// Points to `setup``, helper function
        fn setup_inline(mut self) -> Result<Self, Error> 
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
        /// Calls all required functions to assure the component will not occupy any more resources like pins or network connections, without dropping the value
        fn dismantle(&mut self) -> Result<(), Error> {
            Ok(())
        }

        /// Calls `dismantle()` with an owned object
        fn dismantle_inline(mut self) -> Result<Self, Error> 
        where
            Self : Sized
        {
            self.dismantle()?;
            Ok(self)
        }
    }

    /// A trait that marks a type as a boxing type for another type that includes e.g. GPIO-pins or other physical interfaces that have to be setup
    pub trait Boxed : Setup + Dismantle {
        /// The type that is boxed by this type
        type Boxing;
    }
//