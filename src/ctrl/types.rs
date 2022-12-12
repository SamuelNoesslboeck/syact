use crate::data::StepperData;
use gpio::sysfs::*;

/// Constant for expressing an incorrect pin number
pub const PIN_ERR : u16 = 0xFF;

/// Pin helper enum for safe use in Debug enviroments
#[derive(Debug)]
pub enum RaspPin {
    ErrPin,
    Output(SysFsGpioOutput),
    Input(SysFsGpioInput)
}

/// Different types of enums and their values
#[derive(Debug, Clone)]
pub enum LimitType {
    None,
    Angle(f32),
    Distance(f32)
}

/// Current status of the limits set
#[derive(Debug)]
pub enum LimitDest {
    NoLimitSet,
    NotReached,
    Minimum(f32),
    Maximum(f32)
}

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
    pub fn reached(&self) -> bool {
        match self {
            LimitDest::Maximum(_) => true,
            LimitDest::Minimum(_) => true,
            _ => false
        }
    }
}

/// Update functions for updating stepper data in 
pub enum UpdateFunc {
    None,
    Data(fn (StepperData) -> StepperData, u64),
    Break(for<'a> fn (&'a mut RaspPin) -> bool, u64)
}

/// Result of a stepper operation
pub enum StepResult {
    None,
    Break,
    Error
}