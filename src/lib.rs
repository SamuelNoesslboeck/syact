//! # stepper_lib
//!
//! A library for all types of components used in robots, including controlls for stepper motors, servo motors and more complex assemblies using said motors. Currently all implementations are made for the raspberry pi, though new implementations for more controllers are currently being made.
//!
//! # In Action
//!
//! Let us assume we want to control a simple stepper motor (in this example a [17HE15_1504_S](https://www.omc-stepperonline.com/index.php?route=product/product/get_file&file=2838/17HE15-1504S.pdf)) with a PWM controller connected to the BOARD pins 27 and 19.
//!
//! Importing the library
//!
//! ```toml
//! # ...
//!
//! [dependencies]
//! # Include the library with its standard features
//! stepper_lib = "0.10.6"
//!
//! # ...
//! ```
//!
//! ```rust
//! // Include components and data
//! use crate::{StepperCtrl, StepperConst, SyncComp};
//! use crate::data::LinkedData;
//! // Include the unit system
//! use crate::units::*;
//!
//! // Pin declerations (BOARD on raspberry pi)
//! const PIN_DIR : u8 = 27;
//! const PIN_STEP : u8 = 19;
//!
//! // Define distance and max speed
//! const DELTA : Delta = Delta(2.0 * PI);
//! const OMEGA : Omega = Omega(10.0);
//!
//! fn main() -> Result<(), crate::Error> {
//!     // Create the controls for a stepper motor
//!     let mut ctrl = StepperCtrl::new(StepperConst::MOT_17HE15_1504S, PIN_DIR, PIN_STEP);
//!     // Link the component to a system
//!     ctrl.write_link(LinkedData { 
//!         u: 12.0,    // System voltage in volts
//!         s_f: 1.5    // System safety factor, should be at least 1.0
//!     }); 
//!
//!     // Apply some loads
//!     ctrl.apply_inertia(Inertia(0.2));
//!     ctrl.apply_force(Force(0.10));
//!
//!     println!("Staring to move");
//!     ctrl.drive_rel(DELTA, OMEGA)?;      // Move the motor
//!     println!("Distance {}rad with max speed {:?}rad/s done", DELTA, OMEGA);
//!
//!     Ok(())
//! }
//! ```
#![crate_name = "stepper_lib"]
#![cfg_attr(not(feature = "std"), no_std)]
// #![deny(missing_docs)]

// Modules
#[cfg(not(feature = "embedded"))]
extern crate alloc;

/// Components including stepper motors
pub mod comp;

/// Collection of structs and functions for controlling Stepper Motors
pub mod ctrl;
/// Structs for storing characteristics of stepper motors and devices
pub mod data;
/// Functions and Structs for calculating Stepper Motor procedures and operations
pub mod math;
/// Functions and Structs for taking measurements with a robot for e.g. position calculation
pub mod meas;

/// Self defined units for mathematical operations
pub mod units;

/// Module with all the tests required to assure the library funcitons as intended
#[cfg(test)]
mod tests;

// Wrapped types
/// The general error type used in the crate
#[cfg(feature = "std")]
pub type Error = std::io::Error;

#[cfg(not(feature = "std"))]
pub type Error = ErrorKind;

// Relocated types
pub use comp::{SyncComp, SyncCompGroup, Tool};
pub use ctrl::StepperCtrl;
pub use data::StepperConst;

// Library Types
pub use glam::{Vec3, Mat3};

#[cfg(not(feature = "std"))]
#[derive(Debug)]
pub enum ErrorKind {
    // Components
    NoSuper,
    
    // Load
    Overload
}