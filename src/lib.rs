#![doc = include_str!("../README.md")]
#![crate_name = "syact"]
#![cfg_attr(not(feature = "std"), no_std)]
// #![warn(missing_docs)]

// Modules
#[cfg(feature = "std")]
extern crate alloc;

// ####################
// #    SUBMODULES    #
// ####################
    // # Basic
    // 
    // Contains all the basic content that is available without being hosted on a operating system like raspbian on the raspberry pi
        /// Collection of structs and functions for controlling Stepper Motors
        pub mod ctrl;
        pub use ctrl::{Direction, Stepper};
    //

    // # STD-Content
    // 
    // Content that is only available if the library is hosted on an operating system
        cfg_if::cfg_if! { if #[cfg(feature = "std")] {
            // Includes documentation from readme file
            #[doc = include_str!("../docs/components.md")]
            pub mod comp;
            pub use comp::{SyncComp, SyncCompGroup};
            pub use comp::asyn::AsyncComp;
            
            // Include proc_macro
            pub use syact_macros::SyncCompGroup;

            /// Structs for storing characteristics of stepper motors and devices
            pub mod data;
            pub use data::{StepperConst, CompData, CompVars};

            /// Functions and Structs for calculating Stepper Motor procedures and operations
            pub mod math;

            /// Functions and Structs for taking measurements with a robot for e.g. position calculation
            pub mod meas;

            #[doc = "../docs/tools.md"]
            pub mod tool;
            pub use tool::{Tool, SimpleTool};

            /// Self defined units for mathematical operations
            #[doc = include_str!("../docs/unit_system.md")]
            pub mod units;
        }}
    // 

    /// Easy import of the functionalities
    pub mod prelude;

    /// Module with all the tests required to assure the library funcitons as intended
    #[cfg(test)]
    #[cfg(feature = "std")]
    mod tests;
// 

// ################
// #    ERRORS    #
// ################
// 
// Different platforms require different types of errors 
//
    cfg_if::cfg_if! { if #[cfg(feature = "std")] {
        // Wrapped types
        /// The general error type used in the crate
        pub type Error = Box<dyn std::error::Error>;

        #[inline(always)]
        fn lib_error<E>(error : E) -> crate::Error 
        where
            E: Into<Box<dyn std::error::Error + Sync + Send>> {
            error.into()
        }
    } else {
        /// The general error type of the crate
        pub type Error = ErrorKind;

        /// Enum for different error types
        #[derive(Debug, Clone, Copy)]
        pub enum ErrorKind {
            // TODO
        }
    }}
//

// ###########################
// #    SETUP & DISMANTLE    #
// ###########################
    /// A trait that provides a universal setup function
    /// 
    /// # Pin management
    /// 
    /// For dynamic initialization purposes, all pin creations should run in a `setup()` function
    pub trait Setup {
        /// Calls all required functions to assure the components functionality
        fn setup(&mut self) -> Result<(), Error> { 
            Ok(()) 
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
    }

    /// A trait that marks a type as a boxing type for another type that includes e.g. GPIO-pins or other physical interfaces that have to be setup
    pub trait Boxed : Setup + Dismantle {
        /// The type that is boxed by this type
        type Boxing;
    }
//