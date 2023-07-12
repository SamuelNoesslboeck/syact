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

pub mod stepper;
pub use stepper::{Stepper, Controller};
// 

/// Direction of movement
#[derive(Clone, Copy, PartialEq, PartialOrd, Eq)]
pub enum Direction {
    /// Clockwise
    CW, 
    /// Counterclockwise
    CCW
}

impl Direction {
    pub fn as_bool(self) -> bool {
        match self {
            Direction::CW => true, 
            Direction::CCW => false
        }
    }
}