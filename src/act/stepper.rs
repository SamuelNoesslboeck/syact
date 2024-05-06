use syunit::*;

use crate::data::MicroSteps;
use crate::math::movements::DefinedActuator;
use crate::{StepperConst, SyncActuator, SyncActuatorGroup, StepperConfig};

// Public imports
    pub use syact_macros::StepperActuatorGroup;
// 

// Submodules
    mod builder;
    pub use builder::{DriveError, DriveMode, StepperBuilder, StartStopBuilder};

    mod ctrl;
    pub use ctrl::{Controller, GenericPWM};

    mod motor;
// 

// #####################
// #   Stepper-Types   #
// #####################
    /// Default `Stepper` type for high-performance systems (high resolution stepper)
    pub type Stepper<S, D> = motor::ThreadedStepper<StartStopBuilder, GenericPWM<S, D>>;

    // pub struct MicroSteps(u8);
// 

// #####################
// #    ERROR-TYPES    #
// #####################
    #[derive(Debug, Clone)]
    pub enum StepError {
        TimeTooShort(Time),
        TimeIsIncorrect(Time)
    }

    impl core::fmt::Display for StepError {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            match self {
                Self::TimeTooShort(t) => 
                    f.write_fmt(format_args!("Bad step time! Time given ({}) is smaller than STEP_PULSE_TIME ({})", t, ctrl::STEP_PULSE_TIME)),
                Self::TimeIsIncorrect(t) => 
                    f.write_fmt(format_args!("Bad step time! Time given ({}) is invalid!", t))
            }
        }
    }

    impl std::error::Error for StepError { }
// 

// Stepper traits
    /// A component based on a stepper motor
    pub trait StepperActuator : SyncActuator + DefinedActuator {
        /// Returns the constants of the stepper motor
        fn consts(&self) -> &StepperConst;

        // Config
            fn config(&self) -> &StepperConfig;

            fn set_config(&mut self, config : StepperConfig);
        // 

        // Microstepping
            /// The amount of microsteps in a full step
            fn microsteps(&self) -> MicroSteps;

            /// Set the amount of microsteps in a full step
            fn set_microsteps(&mut self, micro : MicroSteps);
        //

        // Steps
            /// The angular distance of a step considering microstepping
            fn step_ang(&self) -> Delta;
        // 
    }

    /// A group of stepper motor based components
    pub trait StepperActuatorGroup<T, const C : usize> : SyncActuatorGroup<T, C> 
    where 
        T: StepperActuator + ?Sized + 'static
    {
        fn set_config(&mut self, config : StepperConfig) {
            self.for_each_mut(|comp, _| {
                comp.set_config(config.clone())
            });
        }

        /// Returns the amount of microsteps every component uses
        fn microsteps(&self) -> [MicroSteps; C] {
            self.for_each(|comp, _| {
                comp.microsteps()
            })
        }

        /// Sets the amount of microsteps for each motor 
        fn set_micro(&mut self, micro : [MicroSteps; C]) {
            self.for_each_mut(|comp, index| {
                comp.set_microsteps(micro[index]);
            });
        }
    }
// 