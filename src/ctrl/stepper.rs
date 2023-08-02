use core::time::Duration;
use std::time::Instant;

use crate::Direction;
use crate::StepperConst;
use crate::units::*;

// Submodules
cfg_if::cfg_if! { if #[cfg(feature = "std")] {
    mod hr;
    pub use hr::*; 
} }

mod lr;
pub use lr::*;

use super::pin::*;
// 

pub const STEP_PULSE_WIDTH : Time = Time(1.0 / 40000.0);
const STEP_PULSE_DUR : Duration = Duration::from_micros(25);

// #####################
// #   Stepper-Types   #
// #####################
cfg_if::cfg_if! {
    if #[cfg(feature = "std")] {
        /// Default `Stepper` type for high-performance systems (high resolution stepper)
        pub type Stepper = HRStepper<GenericPWM>;

        impl Stepper {
            /// Creates a new generic stepper with both pins set to [pin::ERR_PIN] just for simulation and testing purposes
            #[inline]
            pub fn new_gen() -> Self {
                Self::new(GenericPWM::new_gen(), StepperConst::GEN)
            }
        }
    } else {
        /// Default `Stepper` type for low-level, no-std hardware environments (low resolution stepper)
        pub type Stepper = LRStepper;
    }
}

/// A controller for the logics of a stepper motor
pub trait Controller {
    fn step_with(&mut self, t_len : Time, t_pause : Time);

    fn step(&mut self, time : Time) {
        if time < (STEP_PULSE_WIDTH * 2.0) {
            self.step_with(STEP_PULSE_WIDTH, STEP_PULSE_WIDTH);
        } else {
            self.step_with(STEP_PULSE_WIDTH, time - STEP_PULSE_WIDTH)
        }
    }

    fn step_no_wait(&mut self, t_pause : Time);

    fn dir(&self) -> Direction;

    fn set_dir(&mut self, dir : Direction);
}

#[derive(Debug)]
pub struct GenericPWM {
    dir : Direction,

    pin_step : UniOutPin,
    pin_dir : UniOutPin,

    t_pause : Duration,
    pause_stamp : Instant
}

impl GenericPWM {
    pub fn new(pin_step : u8, pin_dir : u8) -> Result<Self, crate::Error> {
        Ok(Self {
            dir: Direction::CW,
            
            pin_step: UniPin::new(pin_step)?.into_output(),
            pin_dir: UniPin::new(pin_dir)?.into_output(),

            t_pause: Duration::ZERO,
            pause_stamp: Instant::now()
        }) 
    }

    pub fn new_gen() -> Self {
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
    fn step_with(&mut self, t_len : Time, t_pause : Time) {
        self.pin_step.set_high();
        spin_sleep::sleep(t_len.into());
        self.pin_step.set_low();
        spin_sleep::sleep(t_pause.into());
    }

    fn step_no_wait(&mut self, t_total : Time) {
        let elapsed = self.pause_stamp.elapsed();
        if elapsed < self.t_pause {
            spin_sleep::sleep(self.t_pause - elapsed);      // Makes t_pause = elapsed
        }

        if self.t_pause < STEP_PULSE_DUR {
            spin_sleep::sleep(STEP_PULSE_DUR - self.t_pause);
        }

        self.pin_step.set_high();
        spin_sleep::sleep(STEP_PULSE_DUR);
        self.pin_step.set_low();

        self.pause_stamp = Instant::now();
        self.t_pause = (t_total - STEP_PULSE_WIDTH).into();
    }

    fn dir(&self) -> Direction {
        self.dir
    }

    fn set_dir(&mut self, dir : Direction) {
        self.dir = dir;
        self.pin_dir.set(self.dir.as_bool());
    }
}