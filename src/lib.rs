#![doc = include_str!("../README.md")]
#![crate_name = "stepper_lib"]
#![cfg_attr(not(feature = "std"), no_std)]
// #![warn(missing_docs)]

// Modules
#[cfg(feature = "std")]
extern crate alloc;

// Submodules
    #[doc = include_str!("../docs/components.md")]
    pub mod comp;
    pub use comp::{SyncComp, SyncCompGroup, Tool};
    pub use comp::tool::SimpleTool;
    #[cfg(feature = "std")]
    pub use comp::asyn::{AsyncComp, Direction};

    /// Collection of structs and functions for controlling Stepper Motors
    pub mod ctrl;
    pub use ctrl::StepperCtrl;

    /// Structs for storing characteristics of stepper motors and devices
    pub mod data;
    pub use data::{StepperConst, LinkedData};

    /// General error type of the crate
    pub mod err;

    /// Functions and Structs for calculating Stepper Motor procedures and operations
    pub mod math;

    /// Functions and Structs for taking measurements with a robot for e.g. position calculation
    pub mod meas;

    /// Self defined units for mathematical operations
    pub mod units;

    /// Module with all the tests required to assure the library funcitons as intended
    #[cfg(test)]
    mod tests;
// 

// Wrapped types
/// The general error type used in the crate
#[cfg(feature = "std")]
pub type Error = Box<dyn std::error::Error>;

#[cfg(not(feature = "std"))]
pub type Error = ErrorKind;


#[cfg(not(feature = "std"))]
#[derive(Debug)]
pub enum ErrorKind {
    // Components
    NoSuper,
    
    // Load
    Overload
}

/// A trait that provides a universal setup function
pub trait Setup {
    /// Calls all required functions to assure the components functionality
    fn setup(&mut self) -> Result<(), Error>;
}

#[inline(always)]
fn lib_error<E>(error : E) -> crate::Error 
where
    E: Into<Box<dyn std::error::Error + Sync + Send>> {
    error.into()
}