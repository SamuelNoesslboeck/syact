use syunit::*;

use crate::math::movements::DefinedActuator;
use crate::{StepperConst, ActuatorVars, StepperConfig};
use crate::act::stepper::{ControllerError, Controller};
use crate::data::MicroSteps;
use crate::math;

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
        /// The maximum velocity given is too high
        VelocityMaxTooHigh(Velocity, Velocity),
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

pub trait StepperBuilder : Iterator<Item = Time> {
    // General constructor
        fn new(consts : StepperConst) -> Self
        where 
            Self: Sized;
    // 

    // Data
        fn consts(&self) -> &StepperConst;

        fn vars(&self) -> &ActuatorVars;

        fn config(&self) -> &StepperConfig;

        fn set_config(&mut self, config : StepperConfig);

        fn microsteps(&self) -> MicroSteps;

        fn set_microsteps(&mut self, microsteps : MicroSteps);

        fn step_angle(&self) -> Delta;

        fn dir(&self) -> Direction;

        fn set_overload_curret(&mut self, current : Option<f32>) -> Result<(), DriveError>;
    //

    // Loads
        fn apply_gen_force(&mut self, force : Force) -> Result<(), DriveError>;

        /// Value positive in CW direction
        fn apply_dir_force(&mut self, force : Force) -> Result<(), DriveError>;

        fn apply_inertia(&mut self, inertia : Inertia);
    //

    // Velocity max
        fn velocity_max(&self) -> Velocity;

        fn set_velocity_max(&mut self, velocity  : Velocity) -> Result<(), DriveError>;
    // 

    // Regulation
        fn drive_mode(&self) -> &DriveMode;

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
        velocity_start_stop : Velocity,
        _velocity_max : Option<Velocity>,

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

            // Reset velocity  max if it is too high
            if let Some(velocity_max) = self._velocity_max {
                if velocity_max >= self.velocity_start_stop {
                    self._velocity_max = None;
                }
            }

            Ok(())
        }
    }

    impl Iterator for StartStopBuilder {
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
            fn new(consts : StepperConst) -> Self
            where 
                Self: Sized 
            {
                let mut _self = Self {
                    _vars: ActuatorVars::ZERO,
                    _config: StepperConfig::GEN,

                    velocity_start_stop: Velocity::INFINITY,
                    _velocity_max: None,
                    _microsteps: MicroSteps::default(),

                    distance: 0,
                    distance_counter: 0,
                    _step_angle: consts.step_angle(MicroSteps::default()),
                    _dir: Direction::default(),

                    mode: DriveMode::Inactive,

                    _consts: consts
                };

                // TODO: Enable error
                _self.update_start_stop().unwrap();

                _self
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
            self._velocity_max.unwrap_or(self.velocity_start_stop)
        }

        fn set_velocity_max(&mut self, velocity  : Velocity) -> Result<(), DriveError> {
            if velocity > self.velocity_start_stop {
                Err(DriveError::VelocityMaxTooHigh(velocity, self.velocity_start_stop))
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
                        return Err(DriveError::VelocityMaxTooHigh(velocity, self.velocity_max()))
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
            self.velocity_range = math::kin::velocity_start_stop(
                self._vars.force_after_load_lower(
                    self._consts.torque_overload(self._config.overload_current)
                ).ok_or(DriveError::Overload)?, 
                self._vars.inertia_after_load(self._consts.inertia_motor), 
                self._consts.number_steps * self._microsteps
            );

            // Reset velocity  max if it is too high
            if let Some(velocity_max) = self._velocity_max {
                if velocity_max >= self.velocity_range {
                    self._velocity_max = None;
                }
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
            fn new(consts : StepperConst) -> Self
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

                // TODO: Enable error
                _self.update().unwrap();

                _self
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

        fn set_velocity_max(&mut self, velocity  : Velocity) -> Result<(), DriveError> {
            if velocity > self.velocity_range {
                Err(DriveError::VelocityMaxTooHigh(velocity, self.velocity_range))
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
                        return Err(DriveError::VelocityMaxTooHigh(velocity, self.velocity_max()))
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