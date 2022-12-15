use crate::data::StepperData;
use gpio::sysfs::*;

/// Constant for expressing an incorrect pin number
pub const PIN_ERR : u16 = 0xFF;

/// Pin helper enum for safe use in debug enviroments
#[derive(Debug)]
pub enum RaspPin {
    /// Pin could not be created / Is not initialized yet
    ErrPin,
    /// Pin used for output
    Output(SysFsGpioOutput),
    /// Pin used for input
    Input(SysFsGpioInput)
}

/// Different types of limits and their values
#[derive(Debug, Clone)]
pub enum LimitType {
    None,
    /// Angle in radians
    Angle(f32),
    /// Distance in mm
    Distance(f32)
}

/// Current status of the stepper motor considering the limits set
#[derive(Debug)]
pub enum LimitDest {
    /// No limit has been set yet
    NoLimitSet,
    /// None of the set limits have been reached yet
    NotReached,
    /// The stepper is below the minimum limit set by the angle stored in the enum
    Minimum(f32),
    /// The stepper is above the maximum limit set by the angle stored in the enum 
    Maximum(f32)
}

// For debug purposes
impl std::fmt::Display for LimitDest {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            LimitDest::NoLimitSet => write!(f, "No limit set"),
            LimitDest::NotReached => write!(f, "No limit reached"),
            LimitDest::Minimum(ang) => write!(f, "In minimum {}", ang),
            LimitDest::Maximum(ang) => write!(f, "In maximum {}", ang) 
        }
    }
}

impl LimitDest {
    /// Checks if the limit has been reached, returns false if not reached or set yet
    pub fn reached(&self) -> bool {
        match self {
            LimitDest::Maximum(_) => true,
            LimitDest::Minimum(_) => true,
            _ => false
        }
    }
}

// TODO: REWORK UPDATE FUNCTIONS
/// Update functions for updating stepper data in 
pub enum UpdateFunc {
    /// No updates during the process
    None,
    /// Modify the stepper data in the process
    Data(fn (StepperData) -> StepperData, u64),
    /// Cause a break in the movement process while moving by returning `true` \
    /// The measurement pin of the stepper motor is accessable through the mutable reference stored in the enum
    Break(for<'a> fn (&'a mut RaspPin) -> bool, u64)
}

/// Result of a stepper movement operation
pub enum StepResult {
    /// No errors occured
    None,
    /// The movement process has been interruped on purpose
    Break,
    /// The movement process caused an error
    Error
}