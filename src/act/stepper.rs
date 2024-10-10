use core::sync::atomic::AtomicBool;
use core::sync::atomic::Ordering::Relaxed;

use atomic_float::AtomicF32;
use syunit::*;

use crate::{StepperConst, SyncActuator, SyncActuatorGroup, StepperConfig};
use crate::act::SyncActuatorState;
use crate::data::MicroSteps;
use crate::math::movements::DefinedActuator;

// Public imports
    pub use syact_macros::StepperActuatorGroup;
// 

// Submodules
    mod builder;
    pub use builder::{BuilderError, DriveMode, StepperBuilder, StartStopBuilder, ComplexBuilder};

    mod ctrl;
    pub use ctrl::{StepperController, GenericPWM, ControllerError};

    mod motor;
// 

// #####################
// #   Stepper-Types   #
// #####################
    /// Default `Stepper` type for high-performance systems (high resolution stepper)
    pub type Stepper<S, D> = motor::StepperMotor<StartStopBuilder, GenericPWM<S, D>>;

    /// Complex `Stepper` type for high-performance systems with greater speed but greater risk and higher calculation complexity
    pub type ComplexStepper<S, D> = motor::StepperMotor<ComplexBuilder, GenericPWM<S, D>>;
// 

// ################################
// #    StepperActuator-Traits    #
// ################################
    /// A component based on a stepper motor
    pub trait StepperActuator : SyncActuator + DefinedActuator {
        /// Returns the constants of the stepper motor
        fn consts(&self) -> &StepperConst;

        // Config
            /// Returns a reference to the `StepperConfig` used by the stepper motor
            fn config(&self) -> &StepperConfig;

            /// Set the config used by the Stepper motor
            fn set_config(&mut self, config : StepperConfig) -> Result<(), BuilderError>;
        // 

        // Microstepping
            /// The amount of microsteps in a full step
            fn microsteps(&self) -> MicroSteps;

            /// Set the amount of microsteps in a full step
            fn set_microsteps(&mut self, micro : MicroSteps) -> Result<(), BuilderError>;
        //

        // Steps
            /// The angular distance of a step considering microstepping
            fn step_ang(&self) -> RelDist;
        // 
    }

    /// A group of stepper motor based components
    pub trait StepperActuatorGroup<T, const C : usize> : SyncActuatorGroup<T, C> 
    where 
        T: StepperActuator + ?Sized + 'static
    {
        /// Set the configuration for multiple stepper motors
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
        fn set_micro(&mut self, micro : [MicroSteps; C]) -> Result<(), BuilderError> {
            self.try_for_each_mut(|comp, index| {
                comp.set_microsteps(micro[index])
            })?;
            Ok(())
        }
    }
// 

// ######################
// #    StepperState    #
// ######################
    pub struct StepperState {
        _gamma : AtomicF32,
        _moving : AtomicBool
    }

    impl StepperState {
        pub fn new() -> Self {
            StepperState {
                _gamma: AtomicF32::new(AbsPos::ZERO.0),
                _moving: AtomicBool::new(false)
            }
        }
    }

    impl SyncActuatorState for StepperState {
        fn gamma(&self) -> AbsPos {
            AbsPos(self._gamma.load(Relaxed))
        }

        fn moving(&self) -> bool {
            self._moving.load(Relaxed)
        }

        fn halt(&self) {
            todo!()
        }
    }
// 