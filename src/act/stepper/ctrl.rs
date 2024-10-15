use syunit::*;

// Constants
pub const STEP_PULSE_TIME : Time = Time(1.0 / 100000.0);     // TODO: Remove minium pulse time
// const STEP_PULSE_DUR : Duration = Duration::from_micros(25);

// #####################
// #    ERROR-TYPES    #
// #####################
    /// Error type for errors that can occur with `StepperDrivers`
    #[derive(Debug, Clone)]
    pub enum StepperControllerError {
        /// The time given is too short for the driver to recognize
        TimeTooShort(Time),
        /// The time given is invalid (`<=0` / `NaN` / ... )
        TimeIsInvalid(Time),
        /// There is an issue with the IO of the controller
        IOError
    }

    impl core::fmt::Display for StepperControllerError {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            match self {
                Self::TimeTooShort(t) => 
                    f.write_fmt(format_args!("Bad step time! Time given ({}) is smaller than STEP_PULSE_TIME ({})", t, STEP_PULSE_TIME)),
                Self::TimeIsInvalid(t) => 
                    f.write_fmt(format_args!("Bad step time! Time given ({}) is invalid!", t)),
                Self::IOError => 
                    f.write_fmt(format_args!("IOError! Something is wrong with the pins or other IO methods"))
            }
        }
    }

    // impl std::error::Error for ControllerError { }
// 

/// A controller for the logics of a stepper motor
pub trait StepperController {
    /// Initializes a step with the given `time`, this function will set the pin to `HIGH` 
    fn step(&mut self, time : Time) -> Result<(), StepperControllerError>;

    /// The movement direction of the motor
    fn direction(&self) -> Direction;

    /// Sets the direction of the motor
    fn set_dir(&mut self, dir : Direction) -> Result<(), StepperControllerError>;
}