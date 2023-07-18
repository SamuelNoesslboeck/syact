use crate::meas::MeasData;

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
pub use stepper::{Stepper, Controller};
// 

/// A function that can be used to interrupt the movement process of a component
pub type Interrupter<'a> = fn (&mut dyn MeasData) -> bool;

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