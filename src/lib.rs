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
use serde::{Serialize, Deserialize};

// Units

use core::ops::{Sub, Add, Div};

/// Represents a time
/// 
/// # Unit
/// 
/// - In seconds
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq, PartialOrd)]
pub struct Time(f32);

impl Div<Time> for Time 
{
    type Output = Omega;

    #[inline(always)]
    fn div(self, rhs: Time) -> Self::Output {
        Omega(self / rhs.0)
    }
}

/// The gamma distance represents the actual distance traveled by the component
/// 
/// # Unit
/// 
/// - Can be either radians or millimeters
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq, PartialOrd)]
pub struct Gamma(f32);

impl Gamma
{
    /// Gamma with zero value
    pub const ZERO : Self = Self(0.0);
}

impl Into<f32> for Gamma 
{
    #[inline(always)]
    fn into(self) -> f32 {
        self.0
    }
}

impl Sub<Gamma> for Gamma
{
    type Output = Delta;
    
    #[inline(always)]
    fn sub(self, rhs: Gamma) -> Self::Output {
        Delta(rhs.0 - self.0)
    }
}

impl Add<Delta> for Gamma
{
    type Output = Gamma;

    #[inline(always)]
    fn add(self, rhs: Delta) -> Self::Output {
        Self(self.0 + rhs.0)
    }
}

/// The phi distance represents the mathematical distance used for calculations
/// 
/// # Unit
/// 
/// - Can be either radians or millimeters
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq, PartialOrd)]
pub struct Phi(f32);

impl Into<f32> for Phi
{
    #[inline(always)]
    fn into(self) -> f32 {
        self.0
    }
}

/// The delta distance represents a relative distance traveled by the 
/// 
/// # Unit
/// 
/// - Can be either radians or millimeters
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq, PartialOrd)]
pub struct Delta(f32);

impl Delta 
{
    /// Creates a new delta distance from a starting point `start` and an endpoint `end`
    #[inline(always)]
    pub fn diff(start : Gamma, end : Gamma) -> Self {
        end - start
    }
}

impl Into<f32> for Delta
{
    #[inline(always)]
    fn into(self) -> f32 {
        self.0
    }
}

impl Add<Gamma> for Delta
{
    type Output = Gamma;

    #[inline(always)]
    fn add(self, rhs: Gamma) -> Self::Output {
        Gamma(self.0 + rhs.0)
    }
}

/// Represents a change rate in distance
/// 
/// # Unit
/// 
/// - Can be either radians per second or millimeters per second
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq, PartialOrd)]
pub struct Omega(f32);

impl Into<f32> for Omega
{
    #[inline(always)]
    fn into(self) -> f32 {
        self.0
    }
}

/// Represents a change rate in velocity
/// 
/// # Unit
/// 
/// - Can be either radians per second^2 or millimeters per second^2
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq, PartialOrd)]
pub struct Alpha(f32); 

impl Into<f32> for Alpha
{
    #[inline(always)]
    fn into(self) -> f32 {
        self.0
    }
}

/// Represents an inertia, slowing down movement processes
/// 
/// # Unit
/// 
/// - Can be either kilogramm or kilogramm times meter^2
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq, PartialOrd, Ord, Eq)]
pub struct Inertia(f32);

impl Inertia
{   
    /// Zero value inertia
    pub const ZERO : Self = Self(0.0);
}

impl Into<f32> for Inertia
{
    #[inline(always)]
    fn into(self) -> f32 {
        self.0
    }
}

impl Sub<Inertia> for Inertia 
{
    type Output = Inertia;

    #[inline(always)]
    fn sub(self, rhs: Inertia) -> Self::Output {
        Inertia(self.0 - rhs.0)
    }
}

/// Represents a force, slowing down movement processes, eventually even overloading the component
/// 
/// 
/// # Unit
/// 
/// - Can be either Newton or Newtonmeter
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq, PartialOrd)]
pub struct Force(f32);

impl Force 
{   
    /// Zero value force
    pub const ZERO : Self = Self(0.0);
}

impl Into<f32> for Force
{
    #[inline(always)]
    fn into(self) -> f32 {
        self.0
    }
}

impl Sub<Force> for Force 
{
    type Output = Force;

    #[inline(always)]
    fn sub(self, rhs: Force) -> Self::Output {
        Force(self.0 - rhs.0)
    }
}

impl Div<Inertia> for Force
{
    type Output = Alpha;

    #[inline(always)]
    fn div(self, rhs: Inertia) -> Self::Output {
        Alpha(self.0 / rhs.0)
    }
}