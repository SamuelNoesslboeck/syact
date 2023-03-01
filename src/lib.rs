//! # Stepper Library
//! 
//! A library for all types of components used in robots, including controlls for stepper motors, servo motors and more complex assemblies using them. 
//! Current implementations are made for the raspberry pi controller, using a basic GPIO library that will be replaced in future releases.
#![crate_name = "stepper_lib"]
#![cfg_attr(not(feature = "std"), no_std)]
// #![deny(missing_docs)]

// Modules

/// Components including stepper motors
#[cfg(feature = "std")]
pub mod comp;

/// I/O of configuration files to parse whole component groups out of text
#[cfg(feature = "std")]
pub mod conf;

/// Collection of structs and functions for controlling Stepper Motors
#[cfg(feature = "std")]
pub mod ctrl;
// Structs for storing characteristics of stepper motors and devices
pub mod data;
/// Resources required to process and generate G-Code
#[cfg(feature = "gcode")]
pub mod gcode;
/// Functions and Structs for calculating Stepper Motor procedures and operations
pub mod math;

/// Self defined units for mathematical operations
pub mod units;

/// Module with all the tests required to assure the library funcitons as intended
#[cfg(test)]
#[cfg(feature = "std")]
mod tests;

// Types
#[cfg(feature = "std")]
pub use comp::{Component, ComponentGroup, LinkedData, Tool};
#[cfg(feature = "std")]
pub use conf::{JsonConfig, MachineConfig};
#[cfg(feature = "std")]
pub use ctrl::{SimpleMeas, StepperCtrl, UpdateFunc};
pub use data::StepperConst;
pub use math::MathActor;
pub use units::*;

// Library Types
pub use glam::{Vec3, Mat3};