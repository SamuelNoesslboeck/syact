use std::fmt::Debug;

use gpio::sysfs::*;

use crate::data::StepperConst;

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

// TODO: REWORK UPDATE FUNCTIONS
/// Update functions for updating stepper data in 
#[derive(Clone)]
pub enum UpdateFunc {
    /// No updates during the process
    None,
    /// Modify the stepper data in the process
    Data(fn (StepperConst) -> StepperConst, u64),
    /// Cause a break in the movement process while moving by returning `true` \
    /// The measurement pin of the stepper motor is accessable through the mutable reference stored in the enum
    Break(for<'a> fn (&'a mut RaspPin) -> bool, u64)
}

impl Debug for UpdateFunc {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            UpdateFunc::Break(_, acc) => f.write_str(format!("UpdateFunc::Break {{ acc: {} }}", acc).as_str()),
            UpdateFunc::Data(_, acc) => f.write_str(format!("UpdateFunc::Data {{ acc: {} }}", acc).as_str()),
            UpdateFunc::None => f.write_str("None")
        }
    }
}

/// Result of a stepper movement operation
#[derive(Debug, Clone)]
pub enum StepResult {
    /// No errors occured
    None,
    /// The movement process has been interruped on purpose
    Break,
    /// The movement process caused an error
    Error
}