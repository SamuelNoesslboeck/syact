//! # Stepper Library
//! 
//! A library for controlling stepper motors with a raspberry pi
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

/// Module with all the tests required to assure the library funcitons as intended
#[cfg(test)]
#[cfg(feature = "std")]
mod tests;

// Types
#[cfg(feature = "std")]
pub use comp::{Component, ComponentGroup, LinkedData, Tool, Gammas, Inertias, Forces};
#[cfg(feature = "std")]
pub use conf::{JsonConfig, MachineConfig};
#[cfg(feature = "std")]
pub use ctrl::{SimpleMeas, StepperCtrl, UpdateFunc};
pub use data::StepperConst;
pub use math::MathActor;

// Library Types
pub use glam::{Vec3, Mat3};


// Units

/// The gamma distance represents the actual distance traveled by the component
/// 
/// # Unit
/// 
/// - Can be either radians or millimeters
pub struct Gamma(f32);

/// The phi distance represents the mathematical distance used for calculations
/// /// 
/// # Unit
/// 
/// - Can be either radians or millimeters
pub struct Phi(f32);

/// Represents a change rate in distance
/// 
/// # Unit
/// 
/// - Can be either radians per second or millimeters per second
pub struct Velocity(f32);

/// Represents an inertia, slowing down movement processes
/// 
/// # Unit
/// 
/// - Can be either kilogramm or kilogramm times meter^2
pub struct Inertia(f32);

/// Represents a force, slowing down movement processes, eventually even overloading the component
/// 
/// 
/// # Unit
/// 
/// - Can be either Newton or Newtonmeter
pub struct Force(f32);