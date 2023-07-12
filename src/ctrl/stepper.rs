use crate::Direction;
use crate::units::*;

// Submodules
#[cfg(feature = "std")]
mod hr;
#[cfg(feature = "std")]
pub use hr::*; 

mod lr;
pub use lr::*;
// 

// #####################
// #   Stepper-Types   #
// #####################
cfg_if::cfg_if! {
    if #[cfg(feature = "std")] {
        pub type Stepper = HRStepper<GenericPWM>;
    } else {
        pub type Stepper = LRStepper;
    }
}

pub trait Controller {
    fn step(&mut self, time : Time);

    fn dir(&self) -> Direction;

    fn set_dir(&mut self, dir : Direction);
}

pub struct GenericPWM {
    dir : Direction
}

impl Controller for GenericPWM {
    fn step(&mut self, time : Time) {
        
    }

    fn dir(&self) -> Direction {
        self.dir
    }

    fn set_dir(&mut self, dir : Direction) {
        self.dir = dir;
    }
}