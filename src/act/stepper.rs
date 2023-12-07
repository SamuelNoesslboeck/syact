use crate::{StepperConst, SyncActuator, SyncCompGroup, StepperConfig};
use crate::act::parent::RatioActuatorParent;
use crate::units::*;

// Public imports
    pub use syact_macros::StepperCompGroup;
// 

// Submodules
    mod ctrl;
    pub use ctrl::{Controller, GenericPWM};

    mod motor;
// 

// #####################
// #   Stepper-Types   #
// #####################
    /// Default `Stepper` type for high-performance systems (high resolution stepper)
    pub type Stepper = motor::HRStepper<GenericPWM>;

    impl Stepper {
        /// Creates a new generic stepper with both pins set to [ERR_PIN](super::pin::ERR_PIN) just for simulation and testing purposes
        #[inline]
        pub fn new_gen() -> Self {
            Self::new(GenericPWM::new_gen(), StepperConst::GEN)
        }
    }

    // pub struct MicroSteps(u8);
// 

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
                    f.write_fmt(format_args!("Bad step time! Time given ({}) is smaller than STEP_PULSE_TIME ({})", t, ctrl::STEP_PULSE_TIME)),
                Self::TimeIsIncorrect(t) => 
                    f.write_fmt(format_args!("Bad step time! Time given ({}) is invalid!", t)),
                Self::Other(err) => err.fmt(f)
            }
        }
    }

    impl std::error::Error for StepError { }
// 

// Stepper traits
    /// A component based on a stepper motor
    pub trait StepperActuator : SyncActuator {
        // Super comp
            /// Returns a reference to the motor of the component (either itself or a sub/parent-component)
            fn motor(&self) -> &dyn StepperMotor;

            /// Returns a mutable reference to the motor of the component (either itself or a sub/parent component)
            fn motor_mut(&mut self) -> &mut dyn StepperMotor;
        // 

        /// Returns the constants of the stepper motor
        fn consts(&self) -> &StepperConst;

        fn config(&self) -> &StepperConfig;

        // Microstepping
            /// The amount of microsteps in a full step
            fn microsteps(&self) -> u8;

            /// Set the amount of microsteps in a full step
            fn set_microsteps(&mut self, micro : u8);
        //

        // Steps
            /// The angular distance of a step considering microstepping
            fn step_ang(&self) -> Delta;
        // 
    }

    /// All path and curve calculations are designed for the 
    pub trait StepperMotor : StepperActuator {
        // Calculation
            /// Returns the max torque of the motor at a given speed `omega`
            fn torque_at_speed(&self, omega : Omega) -> Force;

            /// Returns the max acceleration of the motor at a given speed `omega`
            fn alpha_at_speed(&self, omega : Omega) -> Result<Alpha, crate::Error>;
        // 
    }

    /// A group of stepper motor based components
    pub trait StepperCompGroup<T, const C : usize> : SyncCompGroup<T, C> 
    where T: StepperActuator + ?Sized + 'static
    {
        /// Returns the amount of microsteps every component uses
        fn microsteps(&self) -> [u8; C] {
            self.for_each(|comp, _| {
                comp.microsteps()
            })
        }

        /// Sets the amount of microsteps for each motor 
        fn set_micro(&mut self, micro : [u8; C]) {
            self.for_each_mut(|comp, index| {
                comp.set_microsteps(micro[index]);
            });
        }
    }
// 

// General implementations
    impl<T : RatioActuatorParent + sylo::Enable> StepperActuator for T
    where
        T::Child : StepperActuator
    {
        // Motor
            fn motor(&self) -> &dyn StepperMotor {
                self.child().motor()
            }
            
            fn motor_mut(&mut self) -> &mut dyn StepperMotor {
                self.child_mut().motor_mut()
            }
        // 

        fn consts(&self) -> &StepperConst {
            self.child().consts()
        }

        fn config(&self) -> &StepperConfig {
            self.child().config()
        }

        // Microstepping
            fn microsteps(&self) -> u8 {
                self.child().microsteps()
            }

            fn set_microsteps(&mut self, microsteps : u8) {
                self.child_mut().set_microsteps(microsteps)
            }
        // 

        // Steps
            fn step_ang(&self) -> Delta {
                self.delta_for_parent(self.child().step_ang())
            }
        // 
    }
// 