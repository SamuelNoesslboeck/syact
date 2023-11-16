// # `ctrl`-module
//
// Stable: 0.12.1

use alloc::sync::Arc;
use atomic_float::AtomicF32;

// ####################
// #    SUBMODULES    #
// ####################
    /// Basic DC-Motors
    pub mod dc_motor;
    pub use dc_motor::DcMotor;

    /// Helper functions and structs for deserializing
    mod des;

    /// LEDs and other light sources
    pub mod led;

    /// Universal pin structure
    pub mod pin;

    /// PWM-signal 
    pub mod pwm;

    /// Structs and methods for basic servo motors
    pub mod servo;

    /// Stepper motors and controllers in different resolutions
    pub mod stepper;
    pub use stepper::{Stepper, Controller, GenericPWM};
//

// Interruptor
    /// A trait for structs that help interrupting or watching movement processes, the most common use are measurement systems
    pub trait Interruptor {
        /// Direction of the interruptor
        /// - If `None` the interruptor is not dependent on a movement direction
        /// - If `Some` the interruptor is only active when moving in the given direction
        /// 
        /// ### Temporary dependence
        /// 
        /// If an interruptor was previously triggered by a movement, the control applies a temporary direction that lasts as long as
        /// the interruptor is triggered. Otherwise a not direction dependent switch would block movements completely
        fn dir(&self) -> Option<sylo::Direction>;
        
        fn set_temp_dir(&mut self, dir_opt : Option<sylo::Direction>);

        /// Runs a check of the movement process
        fn check(&mut self, gamma : &Arc<AtomicF32>) -> Option<InterruptReason>;
    }

    /// Reasons why an interrupt was triggered
    #[derive(PartialEq, Eq, PartialOrd, Ord, Debug, Clone, Copy)]
    pub enum InterruptReason {
        EndReached,
        Overload,
        Error
    }
//