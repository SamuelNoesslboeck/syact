#![crate_name = "stepper_lib"]
//! # Stepper Library
//! 
//! A library for controlling stepper motors with a raspberry pi

// Modules

/// Collection of structs and functions for controlling Stepper Motors
pub mod controller;
/// Structs for storing characteristics of stepper motors and devices
pub mod data;
/// Resources required to process and generate G-Code
pub mod gcode;
/// Functions and Structs for calculating Stepper Motor procedures and operations
pub mod math;

// Types

pub type StepperData = data::StepperData;

// 