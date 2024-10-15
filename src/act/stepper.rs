use core::sync::atomic::AtomicBool;
use core::sync::atomic::Ordering::Relaxed;

use atomic_float::AtomicF32;
use syunit::*;

use crate::{SyncActuator, SyncActuatorGroup};
use crate::act::SyncActuatorState;
use crate::act::asyn::AsyncActuatorState;
use crate::data::MicroSteps;
use crate::math::movements::DefinedActuator;

// Public imports
    pub use syact_macros::StepperActuatorGroup;
// 

// Submodules
    mod builder;
    pub use builder::{StepperBuilderError, DriveMode, StepperBuilder, StartStopBuilder, ComplexBuilder, StepperBuilderSimple, StepperBuilderAdvanced};

    mod ctrl;
    pub use ctrl::{StepperController, StepperControllerError};

    mod motor;
    pub use motor::StepperMotor;
// 

// ################################
// #    StepperActuator-Traits    #
// ################################
    /// A component based on a stepper motor
    pub trait StepperActuator : SyncActuator + DefinedActuator {
        // Microstepping
            /// The amount of microsteps in a full step
            fn microsteps(&self) -> MicroSteps;

            /// Set the amount of microsteps in a full step
            fn set_microsteps(&mut self, micro : MicroSteps) -> Result<(), StepperBuilderError>;
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
        /// Returns the amount of microsteps every component uses
        fn microsteps(&self) -> [MicroSteps; C] {
            self.for_each(|comp, _| {
                comp.microsteps()
            })
        }

        /// Sets the amount of microsteps for each motor 
        fn set_micro(&mut self, micro : [MicroSteps; C]) -> Result<(), StepperBuilderError> {
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
    /// The state of a stepper motor, whether it is driving etc.
    pub struct StepperState {
        _abs_pos : AtomicF32,
        _moving : AtomicBool,

        should_halt : AtomicBool,
        should_interrupt : AtomicBool
    }

    impl StepperState {
        /// Creates a new `StepperState`
        pub fn new() -> Self {
            StepperState {
                _abs_pos: AtomicF32::new(AbsPos::ZERO.0),
                _moving: AtomicBool::new(false),

                should_halt : AtomicBool::new(false),
                should_interrupt : AtomicBool::new(false)
            }
        }
    }

    impl SyncActuatorState for StepperState {
        fn abs_pos(&self) -> AbsPos {
            AbsPos(self._abs_pos.load(Relaxed))
        }

        fn moving(&self) -> bool {
            self._moving.load(Relaxed)
        }

        fn halt(&self) {
            self.should_halt.store(true, Relaxed);
        }

        fn interrupt(&self) {
            self.should_interrupt.store(true, Relaxed);
        }
    }

    impl AsyncActuatorState for StepperState {
        fn direction(&self) -> Direction {
            todo!()
        }
    
        fn speed_factor(&self) -> Factor {
            todo!()
        }
    }
// 