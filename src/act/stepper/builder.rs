use syunit::*;

use crate::{StepperConst, StepperConfig};
use crate::act::stepper::{StepperController, StepperControllerError};
use crate::data::{ActuatorVars, MicroSteps};

// ####################
// #    SUBMODULES    #
// ####################
    mod complex;
    pub use complex::ComplexBuilder;

    mod free;

    mod start_stop;
    pub use start_stop::StartStopBuilder;
//

// Constants
    pub const DEFAULT_MAX_SPEED_LEVEL : usize = 10;
// 

/// The drive-mode of the stepper motor
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub enum DriveMode {
    /// Driving with a constant velocity
    /// - 0 - `Velocity`: The constant velocity to drive, positive values mean CW movement
    ConstVelocity(Velocity),
    /// Driving with a constant fraction of the maximum speed
    /// - 0 - `Factor`: The speed factor to use, references maximum 
    /// - 1 - `Direction`: The driving direction
    ConstFactor(Factor, Direction),
    /// Driving a fixed distance
    /// - 0 - `RelDist`: The relative distance to drive
    /// - 1 - `Velocity`: The exit velocity of the movement
    /// - 2 - `Factor`: Factor of maximum possible speed
    FixedDistance(RelDist, Velocity, Factor),
    /// Motor is stopping
    Stop,
    /// Signals that the motor is inactive
    Inactive
}

// #####################
// #    ERROR-TYPES    #
// #####################
    /// Errors that can occur while driving a stepper motor
    #[derive(Clone, Debug)]
    pub enum StepperBuilderError {
        /// The given distance is too short for the motor to stop
        DistanceTooShort(RelDist, u64, u64),

        // Velocity
            /// Bad value for velocity, depending on context
            /// - 0: `Velocity` - The given velocity
            InvalidVelocity(Velocity),
            /// The velocity given is too high, depending on the context 
            /// - 0: `Velocity` - The given velocity
            /// - 1: `Velocity` - The maximum velocity
            VelocityTooHigh(Velocity, Velocity),
        // 

        // Acceleration
            InvalidAcceleration(Acceleration),
        // 

        // Jolt
            InvalidJolt(Jolt),
        // 

        /// The load data given is too high, causing an overload
        Overload,
        /// An error caused by the controller
        Controller(StepperControllerError)
    }

    impl core::fmt::Display for StepperBuilderError {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            f.write_fmt(format_args!("{:?}", self))
        }
    }

    impl From<StepperControllerError> for StepperBuilderError {
        fn from(value: StepperControllerError) -> Self {
            Self::Controller(value)
        }
    }

    // impl core::error::Error for BuilderError { }
//

/// A stepperbuilder creates stepper motor curves
pub trait StepperBuilder : Iterator<Item = Time> {
    // Getters
        /// The current step angle in radians
        fn step_angle(&self) -> RelDist;

        /// The current movement direction
        fn direction(&self) -> Direction;
    //

    // Setters
        /// Setting the overload current for more torque output
        fn set_overload_curret(&mut self, current : Option<f32>) -> Result<(), StepperBuilderError>;
    //

    // Microsteps
        /// Returns the amount of microsteps used by the builder
        fn microsteps(&self) -> MicroSteps;

        /// Set the amount of microsteps used by the builder
        fn set_microsteps(&mut self, microsteps : MicroSteps) -> Result<(), StepperBuilderError>;
    // 

    // Velocity max
        /// Maximum velocity allowed by the user if specified
        fn velocity_max(&self) -> Option<Velocity>;

        /// Set the maximum allowed `Velocity`
        /// 
        /// ## Option
        /// 
        /// Set to `None` if no limit is wished
        fn set_velocity_max(&mut self, velocity_opt : Option<Velocity>) -> Result<(), StepperBuilderError>;
    // 

    // Acceleration
        /// Maximum acceleration that will be allowed, if specified by the user with `set_max_acceleration`
        fn acceleration_max(&self) -> Option<Acceleration>;

        /// Set the maximum allowed `Acceleration`
        /// 
        /// ## Option
        /// 
        /// Set to `None` if no limit is wished
        fn set_acceleration_max(&mut self, acceleration_opt : Option<Acceleration>) -> Result<(), StepperBuilderError>;
    // 

    // Jolt
        /// The maximum jolt, if specified by the user
        fn jolt_max(&self) -> Option<Jolt>;

        /// Set the maximum allowed `Jolt` 
        /// 
        /// ## Option
        /// 
        /// Set to `None` if no limit is wished
        fn set_jolt_max(&mut self, jolt_opt : Option<Jolt>) -> Result<(), StepperBuilderError>;
    // 

    // Regulation
        /// Returns the current `DriveMode`
        fn drive_mode(&self) -> &DriveMode;

        /// Sets the drive mode
        fn set_drive_mode<C : StepperController>(&mut self, mode : DriveMode, ctrl : &mut C) -> Result<(), StepperBuilderError>;
    //   
}

// Extension Traits
pub trait StepperBuilderSimple : StepperBuilder {
    // General constructor
        /// Create a new stepperbuilder
        fn new() -> Result<Self, StepperBuilderError>
        where 
            Self: Sized;
    // 
}

pub trait StepperBuilderAdvanced : StepperBuilder {
    // General constructor
        /// Create a new stepperbuilder
        fn new(consts : StepperConst, config : StepperConfig) -> Result<Self, StepperBuilderError>
        where 
            Self: Sized;
    // 

    // Getters
        /// Returns the `StepperConsts` used by the builder
        fn consts(&self) -> &StepperConst;

        /// Returns the `StepperVars` used by the builder
        fn vars(&self) -> &ActuatorVars;

        /// Returns the `StepperConfig` used by the builder
        fn config(&self) -> &StepperConfig;
    //

    // Setters 
        /// Set the configuration that should be used by the builder
        fn set_config(&mut self, config : StepperConfig) -> Result<(), StepperBuilderError>;
    //

    // Loads
        /// Apply a general force, which works in both directions
        fn apply_gen_force(&mut self, force : Force) -> Result<(), StepperBuilderError>;

        /// Apply a directional force, which only applies in one direction
        /// - Value positive in `CW` direction
        fn apply_dir_force(&mut self, force : Force) -> Result<(), StepperBuilderError>;

        /// Apply an inertia to the builder, slowing down movements
        fn apply_inertia(&mut self, inertia : Inertia) -> Result<(), StepperBuilderError>;
    //
}