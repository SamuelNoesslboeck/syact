use embedded_hal::digital::OutputPin;
use syunit::*;

// Constants
pub const STEP_PULSE_TIME : Time = Time(1.0 / 100000.0);     // TODO: Remove minium pulse time
// const STEP_PULSE_DUR : Duration = Duration::from_micros(25);

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

    // impl std::error::Error for ControllerError { }
// 

/// A controller for the logics of a stepper motor
pub trait StepperController {
    /// Moves a step with the total `time`
    fn step(&mut self, time : Time) -> Result<(), ControllerError>;

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
    pin_dir : D
}

impl<S : OutputPin, D : OutputPin> GenericPWM<S, D> {
    /// Creates a new `GenericPWM` signal from the two pins
    pub fn new(pin_step : S, pin_dir : D) -> Self {
        Self {
            dir: Direction::CW,
            
            pin_step,
            pin_dir
        }
    }
}

impl<S : OutputPin, D : OutputPin> StepperController for GenericPWM<S, D> {
    fn step(&mut self, time : Time) -> Result<(), ControllerError> {
        self.pin_step.set_high().unwrap();
        spin_sleep::sleep((time / 2.0).into());
        self.pin_step.set_low().unwrap();
        spin_sleep::sleep((time / 2.0).into());
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