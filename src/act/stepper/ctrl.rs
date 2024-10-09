use core::time::Duration;
use std::time::Instant;

use embedded_hal::digital::OutputPin;
use syunit::*;

// Constants
pub const STEP_PULSE_TIME : Time = Time(1.0 / 100000.0);     // TODO: Remove minium pulse time
const STEP_PULSE_DUR : Duration = Duration::from_micros(25);

// #####################
// #    ERROR-TYPES    #
// #####################
    /// Error type for errors that can occur with `StepperDrivers`
    #[derive(Debug, Clone)]
    pub enum ControllerError {
        /// The time given is too short for the driver to recognize
        TimeTooShort(Time),
        /// The time given is invalid (`<=0` / `NaN` / ... )
        TimeIsInvalid(Time)
    }

    impl core::fmt::Display for ControllerError {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            match self {
                Self::TimeTooShort(t) => 
                    f.write_fmt(format_args!("Bad step time! Time given ({}) is smaller than STEP_PULSE_TIME ({})", t, STEP_PULSE_TIME)),
                Self::TimeIsInvalid(t) => 
                    f.write_fmt(format_args!("Bad step time! Time given ({}) is invalid!", t))
            }
        }
    }

    impl std::error::Error for ControllerError { }
// 

/// A controller for the logics of a stepper motor
pub trait StepperController {
    /// Moves a step with the pulse time `t_len` and the pause time `t_pause`
    fn step_with(&mut self, t_len : Time, t_pause : Time);

    /// Moves a step with the total `time`
    fn step(&mut self, time : Time) {
        if time < (STEP_PULSE_TIME * 2.0) {
            // TODO: Notify about fallback
            self.step_with(STEP_PULSE_TIME, STEP_PULSE_TIME);
        } else {
            self.step_with(STEP_PULSE_TIME, time - STEP_PULSE_TIME)
        }
    }

    /// Moves a step and compensates waiting time
    fn step_no_wait(&mut self, t_total : Time) -> Result<(), ControllerError>;

    /// Moves a final step without waiting
    fn step_final(&mut self) -> Result<(), ControllerError>{
        self.step_no_wait(2.0 * STEP_PULSE_TIME)
    }

    /// The movement direction of the motor
    fn dir(&self) -> Direction;

    /// Sets the direction of the motor
    fn set_dir(&mut self, dir : Direction);
}

/// A generic stepper driver that uses a PWM-signal for step-input
#[derive(Debug)]
pub struct GenericPWM<S : OutputPin, D : OutputPin> {
    dir : Direction,

    pin_step : S,
    pin_dir : D,

    t_pause : Duration,
    pause_stamp : Instant
}

impl<S : OutputPin, D : OutputPin> GenericPWM<S, D> {
    /// Creates a new `GenericPWM` signal from the two pins
    pub fn new(pin_step : S, pin_dir : D) -> Self {
        Self {
            dir: Direction::CW,
            
            pin_step,
            pin_dir,

            t_pause: Duration::ZERO,
            pause_stamp: Instant::now()
        }
    }
}

impl<S : OutputPin, D : OutputPin> StepperController for GenericPWM<S, D> {
    fn step_with(&mut self, t_len : Time, t_pause : Time) {
        self.pin_step.set_high().unwrap();
        spin_sleep::sleep(t_len.into());
        self.pin_step.set_low().unwrap();
        spin_sleep::sleep(t_pause.into());
    }

    fn step_no_wait(&mut self, t_total : Time) -> Result<(), ControllerError> {
        // TODO: Maybe create fallbacks or move to Result?
        if t_total <= STEP_PULSE_TIME {
            return Err(ControllerError::TimeTooShort(t_total));
        }

        if !t_total.is_normal() {
            return Err(ControllerError::TimeIsInvalid(t_total));
        }

        let elapsed = self.pause_stamp.elapsed();
        if elapsed < self.t_pause {
                spin_sleep::sleep(self.t_pause - elapsed);      // Makes t_pause = elapsed
            }

        if self.t_pause < STEP_PULSE_DUR {
                spin_sleep::sleep(STEP_PULSE_DUR - self.t_pause);
            }

        self.pin_step.set_high().unwrap();
        spin_sleep::sleep(STEP_PULSE_DUR);
        self.pin_step.set_low().unwrap();

        self.pause_stamp = Instant::now();
        self.t_pause = (t_total - STEP_PULSE_TIME).into();

        Ok(())
    }

    fn dir(&self) -> Direction {
        self.dir
    }

    fn set_dir(&mut self, dir : Direction) {
        self.dir = dir;
        self.pin_dir.set_state(self.dir.as_bool().into()).unwrap();
    }
}