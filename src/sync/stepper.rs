use core::sync::atomic::AtomicBool;
use core::sync::atomic::Ordering::Relaxed;

use atomic_float::AtomicF32;
use syunit::*;

use crate::{ActuatorError, SyncActuatorState, SyncActuator, DefinedActuator};
use crate::data::MicroSteps;

// ####################
// #    SUBMODULES    #
// ####################
    #[doc = include_str!("../../documentation/sync/stepper/builder.md")]
    pub mod builder;
    pub use builder::{DriveMode, StepperBuilder, StartStopBuilder, ComplexBuilder, StepperBuilderSimple, StepperBuilderAdvanced};

    mod ctrl;
    pub use ctrl::StepperController;

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
            fn set_microsteps(&mut self, micro : MicroSteps) -> Result<(), ActuatorError>;
        //

        // Steps
            /// The angular distance of a step considering microstepping
            fn step_ang(&self) -> RelDist;
        // 
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
// 