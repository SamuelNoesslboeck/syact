use core::str::FromStr;

use alloc::sync::Arc;
use atomic_float::AtomicF32;

use serde::{Serialize, Deserialize};

// Submodules
/// Basic DC-Motors
#[cfg(feature = "std")]
pub mod dc_motor;

#[cfg(feature = "std")]
pub use dc_motor::DcMotor;

/// Helper functions and structs for deserializing
/// 
/// # Features
/// 
/// Only available if the "std"-feature is available
#[cfg(feature = "std")]
mod des;

/// LEDs and other light sources
pub mod led;

/// Universal pin structure
pub mod pin;

/// PWM-signal 
/// 
/// # Features
/// 
/// Only available if the "std"-feature is available
#[cfg(feature = "std")]
pub mod pwm;

/// Structs and methods for basic servo motors
/// 
/// # Features 
/// 
/// Only available if the "std"-feature is available
#[cfg(feature = "std")]
pub mod servo;

/// Stepper motors and controllers in different resolutions
pub mod stepper;
pub use stepper::{Stepper, Controller, GenericPWM};

/// A trait for structs that help interrupting or watching movement processes, the most common use are measurement systems
pub trait Interruptor {
    /// Direction of the interruptor
    /// - If `None` the interruptor is not dependent on a movement direction
    /// - If `Some` the interruptor is only active when moving in the given direction
    fn dir(&self) -> Option<Direction>;

    /// Runs a check of the movement process
    fn check(&mut self, gamma : &Arc<AtomicF32>) -> Option<InterruptReason>;
}

/// Reasons why an interrupt was triggered
#[derive(PartialEq, Eq, PartialOrd, Ord, Debug, Clone, Copy)]
pub enum InterruptReason {
    EndReached,
    Overload,
    Error
}

/// Direction of movement
#[derive(Clone, Copy, PartialEq, PartialOrd, Eq, Debug, Default, Serialize, Deserialize)]
#[repr(u8)]
pub enum Direction {
    /// Counterclockwise (`false` / `0`)
    CCW,
    /// Clockwise (`true` / `1`)
    #[default]
    CW
}

impl Direction {
    /// Creates a new `Direction` value from a bool
    /// - `true` is `CW`
    /// - `false` is `CCW`
    #[inline]
    pub fn from_bool(b : bool) -> Self {
        if b { Direction::CW } else { Direction::CCW }
    }

    /// Converts the given direction into a bool value for logic signals
    /// - `CW` is `true`
    /// - `CCW` is `false` 
    #[inline]
    pub fn as_bool(self) -> bool {
        match self {
            Direction::CCW => false,
            Direction::CW => true
        }
    }

    /// Parses a new `Direction` value from a `u8`
    /// - `0` is `CCW`
    /// - Everything else is `CW` 
    #[inline]
    pub fn from_u8(u : u8) -> Self {
        if u == 0 { Direction::CCW } else { Direction::CW }
    } 

    /// Converts the given `Direction` into a `u8` value
    #[inline]
    pub fn as_u8(self) -> u8 {
        match self {
            Direction::CCW => 0,
            Direction::CW => 1
        }
    }
}

impl Into<bool> for Direction {
    fn into(self) -> bool {
        self.as_bool()
    }
}

impl From<bool> for Direction {
    fn from(value: bool) -> Self {
        Self::from_bool(value)
    }
}

impl Into<u8> for Direction {
    fn into(self) -> u8 {
        self.as_u8()
    }
}

impl From<u8> for Direction {
    fn from(value: u8) -> Self {
        Self::from_u8(value)
    }
}

impl FromStr for Direction {
    type Err = crate::Error;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        if let Ok(u) = s.parse::<u8>() {
            return Ok(Self::from_u8(u))
        }

        if let Ok(b) = s.parse::<bool>() {
            return Ok(Self::from_bool(b))
        }

        if s.trim() == "CCW" {
            return Ok(Self::CCW)
        }

        if s.trim() == "CW" {
            return Ok(Self::CW)
        }

        Err("Failed to parse `Direction` value! Either use `bool` / `u8` representation or use 'CCW' / 'CW'!".into())
    }
}