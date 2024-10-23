use syunit::*;
use syunit::metric::*;

use crate::{StepperConst, StepperConfig, ActuatorError};
use crate::data::{ActuatorVars, MicroSteps};
use crate::sync::stepper::StepperController;

// ####################
// #    SUBMODULES    #
// ####################
    mod complex;
    pub use complex::ComplexBuilder;

    // mod free;

    mod start_stop;
    pub use start_stop::StartStopBuilder;
//

// Constants
    /// The maximum speed level that is used by default
    pub const DEFAULT_MAX_SPEED_LEVEL : usize = 10;
// 

/// The drive-mode of the stepper motor
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub enum DriveMode {
    /// Driving with a constant velocity
    /// - 0 - [RadPerSecond]: The constant velocity to drive, positive values mean CW movement
    ConstVelocity(RadPerSecond),
    /// Driving with a constant fraction of the maximum speed
    /// - 0 - [Factor]: The speed factor to use, references maximum 
    /// - 1 - `Direction`: The driving direction
    ConstFactor(Factor, Direction),
    /// Driving a fixed distance
    /// - 0 - `Radians`: The relative distance to drive
    /// - 1 - [RadPerSecond]: The exit velocity of the movement
    /// - 2 - [Factor]: Factor of maximum possible speed
    FixedDistance(Radians, RadPerSecond, Factor),
    /// Motor is stopping
    Stop,
    /// Signals that the motor is inactive
    Inactive
}

/// A stepperbuilder creates stepper motor curves
pub trait StepperBuilder : Iterator<Item = Seconds> {
    // Getters
        /// The current step angle in radians
        fn step_angle(&self) -> Radians;

        /// The current movement direction
        fn direction(&self) -> Direction;
    //

    // Setters
        /// Setting the overload current for more torque output
        fn set_overload_curret(&mut self, current : Option<f32>) -> Result<(), ActuatorError>;
    //

    // Microsteps
        /// Returns the amount of microsteps used by the builder
        fn microsteps(&self) -> MicroSteps;

        /// Set the amount of microsteps used by the builder
        fn set_microsteps(&mut self, microsteps : MicroSteps) -> Result<(), ActuatorError>;
    // 

    // Velocity
        /// Maximum velocity allowed by the user if specified
        fn velocity_max(&self) -> Option<RadPerSecond>;

        /// Set the maximum allowed [RadPerSecond]
        /// 
        /// ## Option
        /// 
        /// Set to `None` if no limit is wished
        fn set_velocity_max(&mut self, velocity_opt : Option<RadPerSecond>) -> Result<(), ActuatorError>;
    // 

    // Acceleration
        /// Maximum acceleration that will be allowed, if specified by the user with `set_max_acceleration`
        fn acceleration_max(&self) -> Option<RadPerSecond2>;

        /// Set the maximum allowed [RadPerSecond2]
        /// 
        /// ## Option
        /// 
        /// Set to `None` if no limit is wished
        fn set_acceleration_max(&mut self, acceleration_opt : Option<RadPerSecond2>) -> Result<(), ActuatorError>;
    // 

    // Jolt
        /// The maximum jolt, if specified by the user
        fn jolt_max(&self) -> Option<RadPerSecond3>;

        /// Set the maximum allowed [RadPerSecond3]
        /// 
        /// ## Option
        /// 
        /// Set to `None` if no limit is wished
        fn set_jolt_max(&mut self, jolt_opt : Option<RadPerSecond3>) -> Result<(), ActuatorError>;
    // 

    // Regulation
        /// Returns the current `DriveMode`
        fn drive_mode(&self) -> &DriveMode;

        /// Sets the drive mode
        fn set_drive_mode<C : StepperController>(&mut self, mode : DriveMode, ctrl : &mut C) -> Result<(), ActuatorError>;
    //   
}

// Extension Traits
/// Defines a general constructor for the stepper builder, using no motor or configuration data
pub trait SimpleStepperBuilder : StepperBuilder {
    // General constructor
        /// Create a new stepperbuilder
        fn new() -> Result<Self, ActuatorError>
        where 
            Self: Sized;
    // 
}

/// Defines a general constructor for the stepper builder using motor and configuration data
/// 
/// Also the `StepperBuilder`
pub trait AdvancedStepperBuilder : StepperBuilder {
    // General constructor
        /// Create a new stepperbuilder
        fn new(consts : StepperConst, config : StepperConfig) -> Result<Self, ActuatorError>
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
        fn set_config(&mut self, config : StepperConfig) -> Result<(), ActuatorError>;
    //

    // Loads
        /// Apply a general force, which works in both directions
        fn apply_gen_force(&mut self, force : NewtonMeters) -> Result<(), ActuatorError>;

        /// Apply a directional force, which only applies in one direction
        /// - Value positive in `CW` direction
        fn apply_dir_force(&mut self, force : NewtonMeters) -> Result<(), ActuatorError>;

        /// Apply an inertia to the builder, slowing down movements
        fn apply_inertia(&mut self, inertia : KgMeter2) -> Result<(), ActuatorError>;
    //
}