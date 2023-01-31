#![crate_name = "stepper_lib"]
//! # Stepper Library
//! 
//! A library for controlling stepper motors with a raspberry pi

// Modules

/// Components including stepper motors
pub mod comp;

/// I/O of configuration files to parse whole component groups out of text
pub mod conf;

/// Collection of structs and functions for controlling Stepper Motors
pub mod ctrl;
// Structs for storing characteristics of stepper motors and devices
pub mod data;
/// Resources required to process and generate G-Code
pub mod gcode;
/// Functions and Structs for calculating Stepper Motor procedures and operations
pub mod math;

/// Module with all the tests required to assure the library funcitons as intended
#[cfg(test)]
mod tests;

// Types
pub use comp::{Component, ComponentGroup, LinkedData};
pub use conf::JsonConfig;
pub use ctrl::{SimpleMeas, StepperCtrl, UpdateFunc};
pub use data::StepperConst;
pub use math::MathActor;

// Library Types

pub use glam::Vec3;

// 

// Error Types 

//