use crate::SyncComp;

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
    /// Runs a check of the movement process
    fn check(&mut self, comp : &mut dyn SyncComp) -> bool;
}

/// Direction of movement
#[derive(Clone, Copy, PartialEq, PartialOrd, Eq, Debug, Default)]
pub enum Direction {
    /// Counterclockwise (`false`)
    CCW,
    /// Clockwise (`true`)
    #[default]
    CW
}

impl Direction {
    /// Creates a new `Direction` value from a bool
    /// - `true` is `CW`
    /// - `false` is `CCW`
    pub fn from_bool(b : bool) -> Direction {
        if b { Direction::CW } else { Direction::CCW }
    }

    /// Converts the given direction into a bool value for logic signals
    /// - `CW` is `true`
    /// - `CCW` is `false` 
    pub fn as_bool(self) -> bool {
        match self {
            Direction::CCW => false,
            Direction::CW => true
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
        Direction::from_bool(value)
    }
}