use core::time::Duration;
use std::time::Instant;

use sylo::Direction;

use crate::StepperConst;
use crate::units::*;

// Submodules
    mod hr;
    pub use hr::*;

    mod lr;
    pub use lr::*;

    use super::pin::*;
// 

pub const STEP_PULSE_TIME : Time = Time(1.0 / 40000.0);
const STEP_PULSE_DUR : Duration = Duration::from_micros(25);

// #####################
// #   Stepper-Types   #
// #####################
/// Default `Stepper` type for high-performance systems (high resolution stepper)
pub type Stepper = HRStepper<GenericPWM>;

impl Stepper {
    /// Creates a new generic stepper with both pins set to [ERR_PIN](super::pin::ERR_PIN) just for simulation and testing purposes
    #[inline]
    pub fn new_gen() -> Self {
        Self::new(GenericPWM::new_gen(), StepperConst::GEN)
    }
}

/// A controller for the logics of a stepper motor
pub trait Controller {
    fn step_with(&mut self, t_len : Time, t_pause : Time);

    fn step(&mut self, time : Time) {
        if time < (STEP_PULSE_TIME * 2.0) {
            // TODO: Notify about fallback
            self.step_with(STEP_PULSE_TIME, STEP_PULSE_TIME);
        } else {
            self.step_with(STEP_PULSE_TIME, time - STEP_PULSE_TIME)
        }
    }

    fn step_no_wait(&mut self, t_total : Time) -> Result<(), StepError>;

    fn step_final(&mut self) -> Result<(), StepError>{
        self.step_no_wait(2.0 * STEP_PULSE_TIME)
    }

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

    fn step_no_wait(&mut self, t_total : Time) -> Result<(), StepError> {
        // TODO: Maybe create fallbacks or move to Result?
        if t_total <= STEP_PULSE_TIME {
            return Err(StepError::TimeTooShort(t_total));
        }

        if !t_total.is_normal() {
            return Err(StepError::TimeIsIncorrect(t_total));
        }

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
        self.t_pause = (t_total - STEP_PULSE_TIME).into();

        Ok(())
    }

    fn dir(&self) -> Direction {
        self.dir
    }

    fn set_dir(&mut self, dir : Direction) {
        self.dir = dir;
        self.pin_dir.set(self.dir.as_bool());
    }
}

// #####################
// #    ERROR-TYPES    #
// #####################
    #[derive(Debug)]
    pub enum StepError {
        TimeTooShort(Time),
        TimeIsIncorrect(Time),
        Other(crate::Error)
    }

    impl StepError {
        pub fn is_other(&self) -> bool {
            match self {
                Self::Other(_) => true,
                _ => false
            }
        }
    }

    impl core::fmt::Display for StepError {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            match self {
                Self::TimeTooShort(t) => 
                    f.write_fmt(format_args!("Bad step time! Time given ({}) is smaller than STEP_PULSE_TIME ({})", t, STEP_PULSE_TIME)),
                Self::TimeIsIncorrect(t) => 
                    f.write_fmt(format_args!("Bad step time! Time given ({}) is invalid!", t)),
                Self::Other(err) => err.fmt(f)
            }
        }
    }

    impl std::error::Error for StepError { }
// 