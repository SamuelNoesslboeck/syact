use syunit::*;

use crate::math::movements::DefinedActuator;
use crate::{StepperConst, ActuatorVars, StepperConfig};
use crate::act::stepper::{ControllerError, Controller};
use crate::data::MicroSteps;
use crate::math;

// Constants
    pub const DEFAULT_MAX_SPEED_LEVEL : usize = 10;
// 

/// The drive-mode of the stepper motor
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub enum DriveMode {
    /// Driving with a constant velocity
    /// - `Velocity`: The constant velocity to drive
    /// - `Direction`: The direction to drive 
    ConstVelocity(Velocity, Direction),
    /// Driving with a constant fraction of the maximum speed
    /// - `Factor`: The speed factor to use, references maximum 
    ConstFactor(Factor, Direction),
    /// Driving a fixed distance
    FixedDistance(Delta, Velocity, Factor),
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
    pub enum DriveError {
        /// Bad value for velocity cap
        BadVelocity(Velocity),
        /// The target velocity is too high
        TargetVelocityTooHigh(Velocity, Velocity),
        /// The desired exit velocity is too high
        VelocityExitTooHigh(Velocity, Velocity),
        /// The load data given is too high, causing an overload
        Overload,
        /// A limit has been reached
        LimitReached,
        /// An error occured in the controller
        Controller(ControllerError)
    }

    impl std::fmt::Display for DriveError {
        fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            f.write_fmt(format_args!("{:?}", self))
        }
    }

    impl std::error::Error for DriveError { }
//

/// A stepperbuilder creates stepper motor curves
pub trait StepperBuilder : Iterator<Item = Time> {
    // General constructor
        /// Create a new stepperbuilder
        fn new(consts : StepperConst) -> Result<Self, DriveError>
        where 
            Self: Sized;
    // 

    // Data
        /// Returns the `StepperConsts` used by the builder
        fn consts(&self) -> &StepperConst;

        /// Returns the `StepperVars` used by the builder
        fn vars(&self) -> &ActuatorVars;

        /// Returns the `StepperConfig` used by the builder
        fn config(&self) -> &StepperConfig;

        /// Set the configuration that should be used by the builder
        fn set_config(&mut self, config : StepperConfig);

        /// Returns the amount of microsteps used by the builder
        fn microsteps(&self) -> MicroSteps;

        /// Set the amount of microsteps used by the builder
        fn set_microsteps(&mut self, microsteps : MicroSteps);

        /// The current step angle in radians
        fn step_angle(&self) -> Delta;

        /// The current movement direction
        fn dir(&self) -> Direction;

        /// Setting the overload current for more torque output
        fn set_overload_curret(&mut self, current : Option<f32>) -> Result<(), DriveError>;
    //

    // Loads
        /// Apply a general force, which works in both directions
        fn apply_gen_force(&mut self, force : Force) -> Result<(), DriveError>;

        /// Apply a directional force, which only applies in one direction
        /// - Value positive in `CW` direction
        fn apply_dir_force(&mut self, force : Force) -> Result<(), DriveError>;

        /// Apply an inertia to the builder, slowing down movements
        fn apply_inertia(&mut self, inertia : Inertia);
    //

    // Velocity max
        /// Maximum velocity that can be reached by the motor
        fn velocity_max(&self) -> Velocity;

        /// Sets the maximum velocity of the component, operation is ineffective if the component can't even reach the set maximum velocity
        fn set_velocity_cap(&mut self, velocity  : Velocity) -> Result<(), DriveError>;
    // 

    // Regulation
        /// Returns the current `DriveMode`
        fn drive_mode(&self) -> &DriveMode;

        /// Sets the drive mode
        fn set_drive_mode<C : Controller>(&mut self, mode : DriveMode, ctrl : &mut C) -> Result<(), DriveError>;
    //   
}

// ################
// #    COMMON    #
// ################
    pub struct StartStopBuilder {
        _consts : StepperConst,
        _vars : ActuatorVars,
        _config : StepperConfig,

        // Speeds
        /// Start-Stop speed
        velocity_start_stop : Velocity,
        velocity_cap : Option<Velocity>,

        // Cache
        _microsteps : MicroSteps,   
        mode : DriveMode,
        _step_angle : Delta, 
        _dir : Direction,

        distance : u64,
        distance_counter : u64
    }

    impl StartStopBuilder {
        pub fn update_start_stop(&mut self) -> Result<(), DriveError> {
            self.velocity_start_stop = math::kin::velocity_start_stop(
                self._vars.force_after_load_lower(
                    self._consts.torque_overload(self._config.overload_current)
                ).ok_or(DriveError::Overload)?, 
                self._vars.inertia_after_load(self._consts.inertia_motor), 
                self._consts.number_steps * self._microsteps
            );

            Ok(())
        }
    }

    // The iterator yields the values for the stepper motor
    impl Iterator for StartStopBuilder {
        type Item = Time;

        fn next(&mut self) -> Option<Self::Item> {
            match self.mode {
                // Constant velocity
                DriveMode::ConstVelocity(velocity , _) => Some(velocity),
                DriveMode::ConstFactor(factor, _) => Some(self.velocity_max() * factor),
                
                DriveMode::FixedDistance(_, _, factor) => {
                    self.distance_counter += 1;

                    if self.distance_counter > self.distance {
                        self.mode = DriveMode::Inactive;
                        None
                    } else {
                        Some(self.velocity_max() * factor)
                    }
                },
                DriveMode::Stop => {
                    self.mode = DriveMode::Inactive;
                    None
                },
                DriveMode::Inactive => None
            }.map(|velocity | self._consts.step_time(velocity , self._microsteps))
        }
    }

    impl StepperBuilder for StartStopBuilder {
        // General constructors
            fn new(consts : StepperConst) -> Result<Self, DriveError>
            where 
                Self: Sized 
            {
                let mut _self = Self {
                    _vars: ActuatorVars::ZERO,
                    _config: StepperConfig::GEN,

                    velocity_start_stop: Velocity::INFINITY,
                    velocity_cap: None,
                    _microsteps: MicroSteps::default(),

                    distance: 0,
                    distance_counter: 0,
                    _step_angle: consts.step_angle(MicroSteps::default()),
                    _dir: Direction::default(),

                    mode: DriveMode::Inactive,

                    _consts: consts
                };

                // TODO: Enable error
                _self.update_start_stop()?;

                Ok(_self)
            }
        // 

        // Data
            fn consts(&self) -> &StepperConst {
                &self._consts
            }

            fn vars(&self) -> &ActuatorVars {
                &self._vars
            }

            fn config(&self) -> &StepperConfig {
                &self._config
            }
            
            fn set_config(&mut self, config : StepperConfig) {
                self._config = config;
            }

            fn microsteps(&self) -> MicroSteps {
                self._microsteps
            }

            fn set_microsteps(&mut self, microsteps : MicroSteps) {
                // Update step-angle when changing microsteps
                self._step_angle = self._consts.step_angle(microsteps);
                self._microsteps = microsteps;
            }

            fn step_angle(&self) -> Delta {
                self._step_angle
            }

            fn dir(&self) -> Direction {
                self._dir
            }

            fn set_overload_curret(&mut self, current : Option<f32>) -> Result<(), DriveError> {
                self._config.overload_current = current;
                self.update_start_stop()
            }
        // 

        // Loads
            fn apply_gen_force(&mut self, force : Force) -> Result<(), DriveError> {
                self._vars.force_load_gen = force;
                self.update_start_stop()
            }

            fn apply_dir_force(&mut self, force : Force) -> Result<(), DriveError> {
                self._vars.force_load_dir = force;
                self.update_start_stop()
            }
            
            fn apply_inertia(&mut self, inertia : Inertia) {
                self._vars.inertia_load = inertia;
                self.update_start_stop().unwrap();      // Save unwrap
            }
        // 

        fn velocity_max(&self) -> Velocity {
            self.velocity_start_stop.min(
                self.velocity_cap.unwrap_or(Velocity::INFINITY)
            )
        }

        fn set_velocity_cap(&mut self, velocity_cap  : Velocity) -> Result<(), DriveError> {
            if velocity_cap.is_normal() & velocity_cap.is_sign_positive() {
                self.velocity_cap = Some(velocity_cap); 
                Ok(())
            } else {
                Err(DriveError::BadVelocity(velocity_cap))
            }
        }

        fn drive_mode(&self) -> &DriveMode {
            &self.mode
        }

        fn set_drive_mode<C : Controller>(&mut self, mode : DriveMode, ctrl : &mut C) -> Result<(), DriveError> {
            match mode {
                // Driving with a constant velocity
                DriveMode::ConstVelocity(mut velocity, dir) => {
                    velocity = velocity.abs();

                    if velocity > self.velocity_max() {
                        return Err(DriveError::TargetVelocityTooHigh(velocity, self.velocity_max()))
                    } 

                    self._dir = dir;
                    ctrl.set_dir(dir);
                },
                // Drive with a constant factor
                DriveMode::ConstFactor(_, dir) => {
                    self._dir = dir;
                    ctrl.set_dir(dir);
                    // TODO
                },
                DriveMode::FixedDistance(delta, velocity_exit, _) => {
                    if velocity_exit > self.velocity_max() {
                        return Err(DriveError::VelocityExitTooHigh(velocity_exit, self.velocity_max()))
                    }

                    self.distance = self._consts.steps_from_angle_abs(delta, self._microsteps);
                    self.distance_counter = 0;

                    if delta >= Delta::ZERO {
                        self._dir = Direction::CW;
                    } else {
                        self._dir = Direction::CCW;
                    }
                    ctrl.set_dir(self._dir);
                },
                _ => { }
            };

            self.mode = mode;
            Ok(())
        }
    }

    impl DefinedActuator for StartStopBuilder {
        fn ptp_time_for_distance(&self, gamma_0 : Gamma, gamma_t : Gamma) -> Time {
            (gamma_t / gamma_0) / self.velocity_max()
        }
    }


    pub struct ComplexStartStopBuilder {
        _consts : StepperConst,
        _vars : ActuatorVars,
        _config : StepperConfig,

        // Speeds
        velocity_range : Velocity,
        _velocity_max : Option<Velocity>,

        // Cache
        _microsteps : MicroSteps,
        mode : DriveMode,
        _step_angle : Delta, 
        _dir : Direction,

        // Speed levels
        speed_levels : Vec<Velocity>,
        time_sums : Vec<Time>,
        max_speed_level : Option<usize>,
        current_speed_level : usize,

        distance : u64,
        distance_counter : u64
    }

    impl ComplexStartStopBuilder {
        pub fn update(&mut self) -> Result<(), DriveError> {
            let max_speed_level = self.max_speed_level.unwrap_or(DEFAULT_MAX_SPEED_LEVEL);
            let speed_levels : Vec<Velocity> = vec![];

            for i in 0 .. max_speed_level {
                let vel = *self.speed_levels.last().unwrap_or(&Velocity::ZERO);
                let accel = math::force::torque_dyn(self.consts(), vel, self.config().voltage, self.config().overload_current);
                let ( t1, _ ) = math::kin::travel_times(self.step_angle(), vel, accel);
            }

            Ok(())
        }
    }

    impl Iterator for ComplexStartStopBuilder {
        type Item = Time;

        fn next(&mut self) -> Option<Self::Item> {
            match self.mode {
                DriveMode::ConstVelocity(velocity , _) => Some(velocity),
                DriveMode::ConstFactor(factor, _) => Some(self.velocity_max() * factor),
                DriveMode::FixedDistance(_, _, factor) => {
                    self.distance_counter += 1;

                    if self.distance_counter > self.distance {
                        self.mode = DriveMode::Inactive;
                        None
                    } else if (self.distance_counter + self.current_speed_level as u64) > self.distance {
                        self.mode = DriveMode::Stop;
                        self.current_speed_level -= 1;
                        Some(self.speed_levels[self.current_speed_level])
                    } else {
                        Some(self.velocity_max() * factor)
                    }
                },
                DriveMode::Stop => {
                    self.mode = DriveMode::Inactive;
                    None
                },
                DriveMode::Inactive => None
            }.map(|velocity | self._consts.step_time(velocity , self._microsteps))
        }
    }

    impl StepperBuilder for ComplexStartStopBuilder {
        // General constructors
            fn new(consts : StepperConst) -> Result<Self, DriveError>
            where 
                Self: Sized 
            {
                let mut _self = Self {
                    _vars: ActuatorVars::ZERO,
                    _config: StepperConfig::GEN,

                    velocity_range: Velocity::INFINITY,
                    _velocity_max: None,
                    _microsteps: MicroSteps::default(),

                    distance: 0,
                    distance_counter: 0,
                    _step_angle: consts.step_angle(MicroSteps::default()),
                    _dir: Direction::default(),

                    mode: DriveMode::Inactive,

                    time_sums: vec![],
                    speed_levels: vec![],
                    max_speed_level: None,
                    current_speed_level: 0,

                    _consts: consts
                };

                _self.update()?;

                Ok(_self)
            }
        // 

        // Data
            fn consts(&self) -> &StepperConst {
                &self._consts
            }

            fn vars(&self) -> &ActuatorVars {
                &self._vars
            }

            fn config(&self) -> &StepperConfig {
                &self._config
            }
            
            fn set_config(&mut self, config : StepperConfig) {
                self._config = config;
            }

            fn microsteps(&self) -> MicroSteps {
                self._microsteps
            }

            fn set_microsteps(&mut self, microsteps : MicroSteps) {
                self._step_angle = self._consts.step_angle(microsteps);
                self._microsteps = microsteps;
            }

            fn step_angle(&self) -> Delta {
                self._step_angle
            }

            fn dir(&self) -> Direction {
                self._dir
            }

            fn set_overload_curret(&mut self, current : Option<f32>) -> Result<(), DriveError> {
                self._config.overload_current = current;
                self.update()
            }
        // 

        // Loads
            fn apply_gen_force(&mut self, force : Force) -> Result<(), DriveError> {
                self._vars.force_load_gen = force;
                self.update()
            }

            fn apply_dir_force(&mut self, force : Force) -> Result<(), DriveError> {
                self._vars.force_load_dir = force;
                self.update()
            }
            
            fn apply_inertia(&mut self, inertia : Inertia) {
                self._vars.inertia_load = inertia;
                self.update().unwrap();      // Save unwrap
            }
        // 

        fn velocity_max(&self) -> Velocity {
            self._velocity_max.unwrap_or(self.velocity_range)
        }

        fn set_velocity_cap(&mut self, velocity  : Velocity) -> Result<(), DriveError> {
            if velocity > self.velocity_range {
                Err(DriveError::BadVelocity(velocity))
            } else {
                self._velocity_max = Some(velocity ); 
                Ok(())
            }
        }

        fn drive_mode(&self) -> &DriveMode {
            &self.mode
        }

        fn set_drive_mode<C : Controller>(&mut self, mode : DriveMode, ctrl : &mut C) -> Result<(), DriveError> {
            match mode {
                DriveMode::ConstVelocity(velocity, dir) => {
                    if velocity > self.velocity_max() {
                        return Err(DriveError::TargetVelocityTooHigh(velocity, self.velocity_max()))
                    } 

                    self._dir = dir;
                    ctrl.set_dir(dir);
                },
                DriveMode::ConstFactor(_, dir) => {
                    self._dir = dir;
                    ctrl.set_dir(dir);
                },
                DriveMode::FixedDistance(delta, velocity_exit, _) => {
                    if velocity_exit > self.velocity_max() {
                        return Err(DriveError::VelocityExitTooHigh(velocity_exit, self.velocity_max()))
                    }

                    self.distance = self._consts.steps_from_angle_abs(delta, self._microsteps);
                    self.distance_counter = 0;

                    if delta >= Delta::ZERO {
                        self._dir = Direction::CW;
                    } else {
                        self._dir = Direction::CCW;
                    }
                    ctrl.set_dir(self._dir);
                },
                _ => { }
            };

            self.mode = mode;
            Ok(())
        }
    }

    impl DefinedActuator for ComplexStartStopBuilder {
        fn ptp_time_for_distance(&self, gamma_0 : Gamma, gamma_t : Gamma) -> Time {
            (gamma_t / gamma_0) / self.velocity_max()
        }
    }
// 