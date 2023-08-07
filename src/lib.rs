#![doc = include_str!("../README.md")]
#![crate_name = "syact"]
#![cfg_attr(not(feature = "std"), no_std)]
// #![warn(missing_docs)]

// Modules
#[cfg(feature = "std")]
extern crate alloc;

// Submodules
    // Basic
        /// Collection of structs and functions for controlling Stepper Motors
        pub mod ctrl;
        pub use ctrl::{Direction, Stepper};

        /// Easy import of the functionalities
        pub mod prelude;
    //

    // Std use
    cfg_if::cfg_if! { if #[cfg(feature = "std")] {
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
        pub mod units;
    }}

    /// Module with all the tests required to assure the library funcitons as intended
    #[cfg(test)]
    #[cfg(feature = "std")]
    #[path = "../tests/mod.rs"]
    mod tests;
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

/// A trait that provides a universal setup function
/// 
/// # Pin management
/// 
/// For dynamic initialization purposes all pin creations should run in a `setup()` function
pub trait Setup {
    /// Calls all required functions to assure the components functionality
    fn setup(&mut self) -> Result<(), Error> { 
        Ok(()) 
    }
}