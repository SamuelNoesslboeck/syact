#![doc = include_str!("../README.md")]
#![crate_name = "syact"]
#![cfg_attr(not(feature = "std"), no_std)]
#![warn(missing_docs)]

// Modules
#[cfg(feature = "std")]
extern crate alloc;

// Submodules
    #[doc = include_str!("../docs/components.md")]
    #[cfg(feature = "std")]
    pub mod comp;
    #[cfg(feature = "std")]
    pub use comp::{SyncComp, SyncCompGroup, Tool};
    #[cfg(feature = "std")]
    pub use comp::tool::SimpleTool;
    #[cfg(feature = "std")]
    pub use comp::asyn::AsyncComp;
    
    #[cfg(feature = "std")]
    pub use syact_macros::SyncCompGroup;

    /// Collection of structs and functions for controlling Stepper Motors
    pub mod ctrl;
    pub use ctrl::{Direction, Stepper};

    /// Structs for storing characteristics of stepper motors and devices
    #[cfg(feature = "std")]
    pub mod data;
    #[cfg(feature = "std")]
    pub use data::{StepperConst, LinkedData};

    /// Functions and Structs for calculating Stepper Motor procedures and operations
    #[cfg(feature = "std")]
    pub mod math;

    /// Functions and Structs for taking measurements with a robot for e.g. position calculation
    #[cfg(feature = "std")]
    pub mod meas;

    /// Easy import of the functionalities
    pub mod prelude;

    /// Self defined units for mathematical operations
    #[cfg(feature = "std")]
    pub mod units;

    /// Module with all the tests required to assure the library funcitons as intended
    #[cfg(test)]
    #[cfg(feature = "std")]
    mod tests;
// 

// Wrapped types
/// The general error type used in the crate
#[cfg(feature = "std")]
pub type Error = Box<dyn std::error::Error>;

/// The general error type of the crate
#[cfg(not(feature = "std"))]
pub type Error = ErrorKind;

/// Enum for different error types
#[cfg(not(feature = "std"))]
#[derive(Debug)]
pub enum ErrorKind {
    // TODO
}

/// A trait that provides a universal setup function
pub trait Setup {
    /// Calls all required functions to assure the components functionality
    fn setup(&mut self) -> Result<(), Error> { 
        Ok(()) 
    }
}

#[cfg(feature = "std")]
#[inline(always)]
fn lib_error<E>(error : E) -> crate::Error 
where
    E: Into<Box<dyn std::error::Error + Sync + Send>> {
    error.into()
}