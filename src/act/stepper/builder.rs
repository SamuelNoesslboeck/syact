use alloc::vec;
use alloc::vec::Vec;

use syunit::*;

use crate::math::movements::DefinedActuator;
use crate::{StepperConst, ActuatorVars, StepperConfig};
use crate::act::stepper::{ControllerError, StepperController};
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
    pub enum BuilderError {
        /// Bad value for velocity cap
        BadVelocity(Velocity),
        /// The target velocity is too high
        TargetVelocityTooHigh(Velocity, Velocity),
        /// The desired exit velocity is too high
        VelocityExitTooHigh(Velocity, Velocity),
        /// The given distance is too short for the motor to stop
        DistanceTooShort(Delta, u64, u64),
        /// The load data given is too high, causing an overload
        Overload,
        /// A limit has been reached
        LimitReached,
        /// An error occured in the controller
        Controller(ControllerError)
    }

    impl core::fmt::Display for BuilderError {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            f.write_fmt(format_args!("{:?}", self))
        }
    }

    impl core::error::Error for BuilderError { }
//

/// A stepperbuilder creates stepper motor curves
pub trait StepperBuilder : Iterator<Item = Time> {
    // General constructor
        /// Create a new stepperbuilder
        fn new(consts : StepperConst) -> Result<Self, BuilderError>
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

        /// Returns the amount of microsteps used by the builder
        fn microsteps(&self) -> MicroSteps;

        /// The current step angle in radians
        fn step_angle(&self) -> Delta;

        /// The current movement direction
        fn dir(&self) -> Direction;
    //

    // Setters
        /// Set the amount of microsteps used by the builder
        fn set_microsteps(&mut self, microsteps : MicroSteps) -> Result<(), BuilderError>;

        /// Set the configuration that should be used by the builder
        fn set_config(&mut self, config : StepperConfig) -> Result<(), BuilderError>;

        /// Setting the overload current for more torque output
        fn set_overload_curret(&mut self, current : Option<f32>) -> Result<(), BuilderError>;
    //

    // Loads
        /// Apply a general force, which works in both directions
        fn apply_gen_force(&mut self, force : Force) -> Result<(), BuilderError>;

        /// Apply a directional force, which only applies in one direction
        /// - Value positive in `CW` direction
        fn apply_dir_force(&mut self, force : Force) -> Result<(), BuilderError>;

        /// Apply an inertia to the builder, slowing down movements
        fn apply_inertia(&mut self, inertia : Inertia) -> Result<(), BuilderError>;
    //

    // Velocity max
        /// Maximum velocity that can be reached by the motor
        fn velocity_max(&self) -> Velocity;

        /// Sets the maximum velocity of the component, operation is ineffective if the component can't even reach the set maximum velocity
        fn set_velocity_cap(&mut self, velocity  : Velocity) -> Result<(), BuilderError>;
    // 

    // Regulation
        /// Returns the current `DriveMode`
        fn drive_mode(&self) -> &DriveMode;

        /// Sets the drive mode
        fn set_drive_mode<C : StepperController>(&mut self, mode : DriveMode, ctrl : &mut C) -> Result<(), BuilderError>;
    //   
}

// ################
// #    COMMON    #
// ################
    /// A simple builder that moves the stepper motor only in its start-stop-range  
    /// - High performance
    /// - Very safe
    /// - Low movement speed
    /// - Microstepping basically impossible
    #[derive(Debug)]
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
        /// Updates the builders velocity values considering the loads etc.
        pub fn update_start_stop(&mut self) -> Result<(), BuilderError> {
            self.velocity_start_stop = math::kin::velocity_start_stop(
                self._vars.force_after_load_lower(
                    self._consts.torque_overload(self._config.overload_current)
                ).ok_or(BuilderError::Overload)?, 
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
            fn new(consts : StepperConst) -> Result<Self, BuilderError>
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
            
            fn set_config(&mut self, config : StepperConfig) -> Result<(), BuilderError> {
                self._config = config;
                self.update_start_stop()
            }

            fn microsteps(&self) -> MicroSteps {
                self._microsteps
            }

            fn set_microsteps(&mut self, microsteps : MicroSteps) -> Result<(), BuilderError> {
                // Update step-angle when changing microsteps
                self._step_angle = self._consts.step_angle(microsteps);
                self._microsteps = microsteps;
                self.update_start_stop()
            }

            fn step_angle(&self) -> Delta {
                self._step_angle
            }

            fn dir(&self) -> Direction {
                self._dir
            }

            fn set_overload_curret(&mut self, current : Option<f32>) -> Result<(), BuilderError> {
                self._config.overload_current = current;
                self.update_start_stop()
            }
        // 

        // Loads
            fn apply_gen_force(&mut self, force : Force) -> Result<(), BuilderError> {
                self._vars.force_load_gen = force;
                self.update_start_stop()
            }

            fn apply_dir_force(&mut self, force : Force) -> Result<(), BuilderError> {
                self._vars.force_load_dir = force;
                self.update_start_stop()
            }
            
            fn apply_inertia(&mut self, inertia : Inertia) -> Result<(), BuilderError> {
                self._vars.inertia_load = inertia;
                self.update_start_stop()
            }
        // 

        fn velocity_max(&self) -> Velocity {
            self.velocity_start_stop.min(
                self.velocity_cap.unwrap_or(Velocity::INFINITY)
            )
        }

        fn set_velocity_cap(&mut self, velocity_cap : Velocity) -> Result<(), BuilderError> {
            if velocity_cap.is_normal() {
                self.velocity_cap = Some(velocity_cap.abs()); 
                self.update_start_stop()
            } else {
                Err(BuilderError::BadVelocity(velocity_cap))
            }
        }

        fn drive_mode(&self) -> &DriveMode {
            &self.mode
        }

        fn set_drive_mode<C : StepperController>(&mut self, mode : DriveMode, ctrl : &mut C) -> Result<(), BuilderError> {
            match mode {
                // Driving with a constant velocity
                DriveMode::ConstVelocity(mut velocity, dir) => {
                    velocity = velocity.abs();

                    if velocity > self.velocity_max() {
                        return Err(BuilderError::TargetVelocityTooHigh(velocity, self.velocity_max()))
                    } 

                    self._dir = dir;
                    ctrl.set_dir(dir);
                },
                // Drive with a constant factor
                DriveMode::ConstFactor(_, dir) => {
                    self._dir = dir;
                    ctrl.set_dir(dir);
                },
                DriveMode::FixedDistance(delta, velocity_exit, _) => {
                    if velocity_exit > self.velocity_max() {
                        return Err(BuilderError::VelocityExitTooHigh(velocity_exit, self.velocity_max()))
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
            (gamma_t - gamma_0) / self.velocity_max()
        }
    }

    /// A more complex builder that introduces speed levels 
    /// - High movements speeds
    /// - Perfect for microstepping
    #[derive(Debug)]
    pub struct ComplexBuilder {
        _consts : StepperConst,
        _vars : ActuatorVars,
        _config : StepperConfig,

        // Speeds
        _velocity_cap : Option<Velocity>,

        // Cache
        _microsteps : MicroSteps,
        _step_angle : Delta, 
        _dir : Direction,

        // Modes
        mode : DriveMode,
        cached_mode : Option<DriveMode>,

        // Speed levels
        speed_levels : Vec<Velocity>,
        time_sums : Vec<Time>,
        times : Vec<Time>,
        max_speed_level : Option<usize>,
        current_speed_level : usize,

        distance : u64,
        distance_counter : u64
    }

    impl ComplexBuilder {
        /// Updates the builders speed levels and times considering loads etc.
        pub fn update(&mut self) -> Result<(), BuilderError> {
            // Store relevant values
            let max_speed_level = self.max_speed_level.unwrap_or(DEFAULT_MAX_SPEED_LEVEL);
            let velocity_cap = self.velocity_cap();

            // Create new arrays
            let mut speed_levels : Vec<Velocity> = vec![];
            let mut time_sums : Vec<Time> = vec![];
            let mut times : Vec<Time> = vec![];

            let mut vel = Velocity::ZERO;

            // Iterate to max speed level or until the cap is reached
            for _ in 0 .. max_speed_level {
                // Calculate acceleration and movement time when fully accelerating
                let accel = self.consts().alpha_max_for_velocity(self.vars(), self.config(), vel, self.dir())
                    .ok_or(BuilderError::Overload)?;
                let ( mut move_time, _ ) = math::kin::travel_times(self.step_angle(), vel, accel);

                vel += accel * move_time;

                // If the velocity is greater than the cap, recalc values, store them, and break the loop
                if vel > velocity_cap {
                    move_time = 2.0 * self.step_angle() / (*self.speed_levels.last().unwrap_or(&Velocity::ZERO) + velocity_cap);
                    vel = velocity_cap;

                    speed_levels.push(vel);
                    time_sums.push(*time_sums.last().unwrap_or(&Time::ZERO) + move_time);
                    times.push(move_time);

                    break;
                }

                // Store values
                speed_levels.push(vel);
                time_sums.push(*time_sums.last().unwrap_or(&Time::ZERO) + move_time);
                times.push(move_time);
            }

            // Update class values
            self.speed_levels = speed_levels;
            self.times = times;
            self.time_sums = time_sums;

            Ok(())
        }

        /// Returns the cap velocity
        /// - Is either the cap velocity given by the user
        /// - Or the maximum recommended velocity for a stepper motor
        /// depends on which is lower
        pub fn velocity_cap(&self) -> Velocity {
            self._velocity_cap.unwrap_or(Velocity::INFINITY)
                .min(self.consts().velocity_max(self.config().voltage))
        }

        /// Stops the builder with the given drivemode
        pub fn stop_with_mode(&mut self, mode : DriveMode) {
            self.mode = DriveMode::Stop;
            self.cached_mode = Some(mode);
        }
        
        /// The current velocity of the builder
        pub fn velocity_current(&self) -> Velocity {
            self.current_speed_level.checked_sub(1)
                .map(|i| self.speed_levels[i])
                .unwrap_or(Velocity::ZERO)
        }

        /// Moves the builder towards the next speed-level closer to the desired velocity `vel_tar`
        pub fn goto_velocity(&mut self, vel_tar : Velocity) -> Result<Velocity, BuilderError> {
            let vel_below = self.current_speed_level.checked_sub(2)
                .map(|i| self.speed_levels[i])
                .unwrap_or(Velocity::ZERO);

            if vel_tar > self.velocity_current() {
                // Desired velocity is greater than the current speed, increasing speed level if possible
                if let Some(&time) = self.times.get(self.current_speed_level) {
                    self.current_speed_level += 1;
                    Ok(self.consts().velocity(time, self.microsteps()))
                } else {
                    Err(BuilderError::TargetVelocityTooHigh(vel_tar, *self.speed_levels.last().unwrap_or(&Velocity::ZERO)))
                }
            } else if (vel_tar < vel_below) | ((vel_tar == Velocity::ZERO) & (self.current_speed_level > 0)) {
                // Desired velocity is smaller than the speed level BELOW, meaning that it is out of range of this speed level
                self.current_speed_level = self.current_speed_level.saturating_sub(1);
                Ok(self._consts.velocity(self.times[self.current_speed_level], self._microsteps))
            } else {
                // If vel is equal to or in range of speed level, return velocity
                Ok(vel_tar)
            }
        }
    }

    impl Iterator for ComplexBuilder {
        type Item = Time;

        fn next(&mut self) -> Option<Self::Item> {
            let mut vel_opt = match self.mode {
                DriveMode::ConstVelocity(velocity , _) => self.goto_velocity(velocity).ok(),
                DriveMode::ConstFactor(factor, _) => self.goto_velocity(self.velocity_max() * factor).ok(),
                DriveMode::FixedDistance(_, _, factor) => {
                    self.distance_counter += 1;

                    // Special case with only one node
                    if (self.distance == 1) & (self.distance_counter == 1) {
                        return self.times.first().map(|v| *v);
                    }

                    if ((self.distance_counter + self.current_speed_level as u64) == self.distance) & ((self.distance % 2) == 1) {
                        Some(self.speed_levels[self.current_speed_level.saturating_sub(1)])
                    } else if (self.distance_counter + self.current_speed_level as u64) > self.distance {
                        self.goto_velocity(Velocity::ZERO).ok()
                    } else {
                        self.goto_velocity(self.velocity_max() * factor).ok()
                    }
                },
                DriveMode::Stop => {
                    self.goto_velocity(Velocity::ZERO).ok()
                },
                DriveMode::Inactive => None
            };

            if let Some(vel) = vel_opt {
                if vel == Velocity::ZERO {
                    vel_opt = None;
                }
            }

            vel_opt.map(|vel| self._consts.step_time(vel, self._microsteps))
        }
    }

    impl StepperBuilder for ComplexBuilder {
        // General constructors
            fn new(consts : StepperConst) -> Result<Self, BuilderError>
            where 
                Self: Sized 
            {
                let mut _self = Self {
                    _vars: ActuatorVars::ZERO,
                    _config: StepperConfig::GEN,

                    _velocity_cap: None,
                    _microsteps: MicroSteps::default(),

                    distance: 0,
                    distance_counter: 0,
                    _step_angle: consts.step_angle(MicroSteps::default()),
                    _dir: Direction::default(),

                    mode: DriveMode::Inactive,
                    cached_mode: None,

                    time_sums: vec![],
                    times: vec![],
                    speed_levels: vec![],
                    max_speed_level: None,
                    current_speed_level: 0,

                    _consts: consts
                };

                _self.update()?;

                Ok(_self)
            }
        // 

        // Getters
            fn consts(&self) -> &StepperConst {
                &self._consts
            }

            fn vars(&self) -> &ActuatorVars {
                &self._vars
            }

            fn config(&self) -> &StepperConfig {
                &self._config
            }

            fn microsteps(&self) -> MicroSteps {
                self._microsteps
            }

            fn step_angle(&self) -> Delta {
                self._step_angle
            }

            fn dir(&self) -> Direction {
                self._dir
            }
        //

        // Setters
            fn set_microsteps(&mut self, microsteps : MicroSteps) -> Result<(), BuilderError> {
                self._step_angle = self._consts.step_angle(microsteps);
                self._microsteps = microsteps;
                self.update()
            }

            fn set_config(&mut self, config : StepperConfig) -> Result<(), BuilderError> {
                self._config = config;
                self.update() 
            }

            fn set_overload_curret(&mut self, current : Option<f32>) -> Result<(), BuilderError> {
                self._config.overload_current = current;
                self.update()
            }
        // 

        // Loads
            fn apply_gen_force(&mut self, force : Force) -> Result<(), BuilderError> {
                self._vars.force_load_gen = force;
                self.update()
            }

            fn apply_dir_force(&mut self, force : Force) -> Result<(), BuilderError> {
                self._vars.force_load_dir = force;
                self.update()
            }
            
            fn apply_inertia(&mut self, inertia : Inertia) -> Result<(), BuilderError> {
                self._vars.inertia_load = inertia;
                self.update()
            }
        // 

        fn velocity_max(&self) -> Velocity {
            self.velocity_cap().min(
                *self.speed_levels.last().unwrap_or(&Velocity::ZERO)
            )
        }

        fn set_velocity_cap(&mut self, velocity_cap : Velocity) -> Result<(), BuilderError> {
            if velocity_cap.is_normal() {
                self._velocity_cap = Some(velocity_cap.abs()); 
                self.update()
            } else {
                Err(BuilderError::BadVelocity(velocity_cap))
            }
        }

        fn drive_mode(&self) -> &DriveMode {
            &self.mode
        }

        fn set_drive_mode<C : StepperController>(&mut self, mode : DriveMode, ctrl : &mut C) -> Result<(), BuilderError> {
            match mode {
                DriveMode::ConstVelocity(velocity, dir) => {
                    if velocity > self.velocity_max() {
                        return Err(BuilderError::TargetVelocityTooHigh(velocity, self.velocity_max()))
                    } 

                    if (self.mode != DriveMode::Inactive) & (dir != self._dir) {
                        // Turn around motor
                        self.stop_with_mode(mode.clone());
                    } else {
                        self._dir = dir;
                        ctrl.set_dir(dir);
                    }
                },
                DriveMode::ConstFactor(_, dir) => {
                    if (self.mode != DriveMode::Inactive) & (dir != self._dir) {
                        // Turn around motor
                        self.stop_with_mode(mode.clone());
                    } else {
                        self._dir = dir;
                        ctrl.set_dir(dir);
                    }
                },
                DriveMode::FixedDistance(delta, velocity_exit, _) => {
                    if velocity_exit > self.velocity_max() {
                        return Err(BuilderError::VelocityExitTooHigh(velocity_exit, self.velocity_max()))
                    }

                    self.distance = self._consts.steps_from_angle_abs(delta, self._microsteps);
                    self.distance_counter = 0;

                    // // TODO: Remove for full hold
                    // if self.distance > self.current_speed_level as u64 {
                    //     return Err(DriveError::DistanceTooShort(self.step_angle(), self.distance, self.current_speed_level as u64))
                    // }

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
    
    impl DefinedActuator for ComplexBuilder {
        fn ptp_time_for_distance(&self, gamma_0 : Gamma, gamma_t : Gamma) -> Time {
            let delta = gamma_t - gamma_0;
            let distance = self.consts().steps_from_angle_abs(delta, self.microsteps());

            let max_speed_level = (distance / 2).saturating_sub(1);

            // Multiple cases
            if distance == 1 {
                *self.times.first().unwrap_or(&Time::INFINITY)
            } else if max_speed_level < self.speed_levels.len() as u64 {
                self.time_sums[max_speed_level as usize] * 2.0
                    + self.consts().step_time(self.speed_levels[max_speed_level as usize], self.microsteps())
            } else {
                let distance_rest = distance - self.times.len() as u64 * 2;

                self.consts().step_time(self.velocity_max(), self.microsteps()) * distance_rest as f32
                    + *self.time_sums.last().unwrap_or(&Time::ZERO) * 2.0
            }
        }
    }
// 