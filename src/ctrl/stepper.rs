use core::time::Duration;

use crate::Direction;
use crate::StepperConst;
use crate::units::*;

// Submodules
#[cfg(feature = "std")]
mod hr;
#[cfg(feature = "std")]
pub use hr::*; 

mod lr;
pub use lr::*;

use super::pin::ERR_PIN;
use super::pin::UniOutPin;
use super::pin::UniPin;
// 

// #####################
// #   Stepper-Types   #
// #####################
cfg_if::cfg_if! {
    if #[cfg(feature = "std")] {
        /// Default `Stepper` type for high-performance systems (high resolution stepper)
        pub type Stepper = HRStepper<GenericPWM>;

        impl Stepper {
            /// Creates a new structure with both pins set to [pin::ERR_PIN] just for simulation and testing purposes
            #[inline]
            pub fn new_sim() -> Self {
                Self::new(GenericPWM::new_sim(), StepperConst::GEN)
            }
        }
    } else {
        /// Default `Stepper` type for low-level, no-std hardware environments (low resolution stepper)
        pub type Stepper = LRStepper;
    }
}

/// A controller for the logics of a stepper motor
pub trait Controller {
    fn step(&mut self, time : Time);

    fn dir(&self) -> Direction;

    fn set_dir(&mut self, dir : Direction);
}

#[derive(Debug)]
pub struct GenericPWM {
    dir : Direction,

    pin_step : UniOutPin,
    pin_dir : UniOutPin
}

impl GenericPWM {
    pub fn new(pin_step : u8, pin_dir : u8) -> Result<Self, crate::Error> {
        Ok(Self {
            dir: Direction::CW,
            
            pin_step: UniPin::new(pin_step)?.into_output(),
            pin_dir: UniPin::new(pin_dir)?.into_output()
        }) 
    }

    pub fn new_sim() -> Self {
        Self::new(ERR_PIN, ERR_PIN).unwrap()
    }

    pub fn pin_step(&self) -> u8 {
        self.pin_step.pin
    }

    pub fn pin_dir(&self) -> u8 {
        self.pin_dir.pin
    }
}

impl Controller for GenericPWM {
    fn step(&mut self, time : Time) {
        let step_time_half : Duration = (time / 2.0).into(); 

        self.pin_step.set_high();
        spin_sleep::sleep(step_time_half);
        self.pin_step.set_low();
        spin_sleep::sleep(step_time_half);
    }

    fn dir(&self) -> Direction {
        self.dir
    }

    fn set_dir(&mut self, dir : Direction) {
        self.dir = dir;
        self.pin_dir.set(self.dir.as_bool());
    }
}