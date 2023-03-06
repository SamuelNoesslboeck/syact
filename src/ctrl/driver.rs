use core::time::Duration;
use std::sync::Arc;
use std::thread;

use gpio::{GpioIn, GpioOut};
use gpio::sysfs::SysFsGpioOutput;

use crate::data::{LinkedData, StepperConst, StepperVar};
use crate::ctrl::types::*;
use crate::math;

/// ### Driver
/// Driver class for basic stepper motor operations
#[derive(Debug)]
pub struct StepperDriver 
{
    /// Stepper data
    pub consts : StepperConst,
    pub vars : StepperVar,

    /// The current direction of the driver, the bool value is written to the `pin_dir` GPIO pin\
    /// DO NOT WRITE TO THIS VALUE! Use the `Driver::set_dir()` function instead
    pub dir : bool,
    /// The current absolute position since set to a value
    pub pos : i64,

    lk : Arc<LinkedData>,

    /// Pin for defining the direction
    sys_dir : RaspPin,
    /// Pin for PWM Step pulses
    sys_step : RaspPin,
    /// Measurement pin
    sys_meas : RaspPin,

    /// Limit for minimum angle/step count
    limit_min : Option<Gamma>,
    /// Limit for maximum angle/step count
    limit_max : Option<Gamma>
}

impl StepperDriver {
    // Init
    /// Creates a new Driver from the given stepper `StepperData` \
    /// Pin numbers are based on the BCM-Scheme, not by absolute board numbers
    pub fn new(data : StepperConst, pin_dir : u16, pin_step : u16) -> Self {
        // Create pins if possible
        let sys_dir = match SysFsGpioOutput::open(pin_dir.clone()) {
            Ok(val) => RaspPin::Output(val),
            Err(_) => RaspPin::ErrPin
        };

        let sys_step = match SysFsGpioOutput::open(pin_step.clone()) {
            Ok(val) => RaspPin::Output(val),
            Err(_) => RaspPin::ErrPin
        };

        let mut driver = StepperDriver {
            consts: data, 
            vars: StepperVar::ZERO, 

            dir: true, 
            pos: 0,

            lk: Arc::new(LinkedData::EMPTY),
            
            sys_dir,
            sys_step,
            sys_meas: RaspPin::ErrPin,

            limit_min: None,
            limit_max: None
        };

        // Write initial direction to output pins
        driver.set_dir(driver.dir);

        driver
    }
}
