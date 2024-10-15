use alloc::vec;
use alloc::vec::Vec;

use syunit::*;

use crate::{StepperConst, StepperConfig};
use crate::act::stepper::{StepperController, StepperControllerError};
use crate::data::{ActuatorVars, MicroSteps};
use crate::math;
use crate::math::movements::DefinedActuator;

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

        /// Returns the amount of microsteps used by the builder
        fn microsteps(&self) -> MicroSteps;

        /// The current step angle in radians
        fn step_angle(&self) -> RelDist;

        /// The current movement direction
        fn direction(&self) -> Direction;
    //

    // Setters
        /// Set the amount of microsteps used by the builder
        fn set_microsteps(&mut self, microsteps : MicroSteps) -> Result<(), StepperBuilderError>;

        /// Set the configuration that should be used by the builder
        fn set_config(&mut self, config : StepperConfig) -> Result<(), StepperBuilderError>;

        /// Setting the overload current for more torque output
        fn set_overload_curret(&mut self, current : Option<f32>) -> Result<(), StepperBuilderError>;
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

    // Velocity max
        /// Maximum velocity that is possible, this value is either:
        /// - The maximum velocity the component can currently reach
        /// - The maximum velocity currently allowed by the user
        /// depending on which one is lower
        fn velocity_possible(&self) -> Velocity;

        /// Maximum velocity allowed by the user if specified
        fn velocity_max(&self) -> Option<Velocity>;

        /// Sets the maximum velocity of the component, operation is ineffective if the component can't even reach the set maximum velocity
        fn set_velocity_max(&mut self, velocity_opt : Option<Velocity>) -> Result<(), StepperBuilderError>;
    // 

    // Acceleration
        /// Maximum acceleration that will be allowed, if specified by the user with `set_max_acceleration`
        fn acceleration_max(&self) -> Option<Acceleration>;

        /// Set the maximum allowed `Acceleration`
        fn set_acceleration_max(&mut self, acceleration_opt : Option<Acceleration>) -> Result<(), StepperBuilderError>;
    // 

    // Regulation
        /// Returns the current `DriveMode`
        fn drive_mode(&self) -> &DriveMode;

        /// Sets the drive mode
        fn set_drive_mode<C : StepperController>(&mut self, mode : DriveMode, ctrl : &mut C) -> Result<(), StepperBuilderError>;
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
        _velocity_max : Option<Velocity>,

        // Acceleration
        _acceleration_max : Option<Acceleration>,

        // Cache
        _microsteps : MicroSteps,   
        mode : DriveMode,
        _step_angle : RelDist, 
        _direction : Direction,

        distance : u64,
        distance_counter : u64
    }

    impl StartStopBuilder {
        /// Updates the builders velocity values considering the loads etc.
        pub fn update_start_stop(&mut self) -> Result<(), StepperBuilderError> {
            self.velocity_start_stop = math::kin::velocity_start_stop(
                self._vars.force_after_load_lower(
                    self._consts.torque_overload(self._config.overload_current)
                ).ok_or(StepperBuilderError::Overload)?, 
                self._vars.inertia_after_load(self._consts.inertia_motor), 
                self._consts.number_steps * self._microsteps
            );

            Ok(())
        }

        /// The maximum velocity that can be reached with the given acceleration
        pub fn velocity_by_max_acceleration(&self) -> Option<Velocity> {
            self.acceleration_max().map(|acceleration_max| {
                math::kin::accel_from_zero_velocity(self.step_angle(), acceleration_max)
            })
        }
    }

    // The iterator yields the values for the stepper motor
    impl Iterator for StartStopBuilder {
        type Item = Time;

        fn next(&mut self) -> Option<Self::Item> {
            match self.mode {
                // Constant velocity
                DriveMode::ConstVelocity(velocity , _) => Some(velocity),
                DriveMode::ConstFactor(factor, _) => Some(self.velocity_possible() * factor),
                
                DriveMode::FixedDistance(_, _, factor) => {
                    self.distance_counter += 1;

                    if self.distance_counter > self.distance {
                        self.mode = DriveMode::Inactive;
                        None
                    } else {
                        Some(self.velocity_possible() * factor)
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
            fn new(consts : StepperConst, config : StepperConfig) -> Result<Self, StepperBuilderError>
            where 
                Self: Sized 
            {
                let mut _self = Self {
                    _vars: ActuatorVars::ZERO,
                    _config: config,

                    velocity_start_stop: Velocity::INFINITY,
                    _velocity_max: None,

                    _microsteps: MicroSteps::default(),
                    _acceleration_max: None,

                    distance: 0,
                    distance_counter: 0,
                    _step_angle: consts.step_angle(MicroSteps::default()),

                    _direction: Direction::default(),

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
            
            fn set_config(&mut self, config : StepperConfig) -> Result<(), StepperBuilderError> {
                self._config = config;
                self.update_start_stop()
            }

            fn microsteps(&self) -> MicroSteps {
                self._microsteps
            }

            fn set_microsteps(&mut self, microsteps : MicroSteps) -> Result<(), StepperBuilderError> {
                // Update step-angle when changing microsteps
                self._step_angle = self._consts.step_angle(microsteps);
                self._microsteps = microsteps;
                self.update_start_stop()
            }

            fn step_angle(&self) -> RelDist {
                self._step_angle
            }

            fn direction(&self) -> Direction {
                self._direction
            }

            fn set_overload_curret(&mut self, current : Option<f32>) -> Result<(), StepperBuilderError> {
                self._config.overload_current = current;
                self.update_start_stop()
            }
        // 

        // Loads
            fn apply_gen_force(&mut self, force : Force) -> Result<(), StepperBuilderError> {
                self._vars.force_load_gen = force;
                self.update_start_stop()
            }

            fn apply_dir_force(&mut self, force : Force) -> Result<(), StepperBuilderError> {
                self._vars.force_load_dir = force;
                self.update_start_stop()
            }
            
            fn apply_inertia(&mut self, inertia : Inertia) -> Result<(), StepperBuilderError> {
                self._vars.inertia_load = inertia;
                self.update_start_stop()
            }
        // 

        // Velocity
            #[inline]
            fn velocity_possible(&self) -> Velocity {
                self.velocity_start_stop.min(
                    self._velocity_max.unwrap_or(Velocity::INFINITY)
                )
            }
            
            #[inline]
            fn velocity_max(&self) -> Option<Velocity> {
                self._velocity_max
            }

            fn set_velocity_max(&mut self, velocity_opt : Option<Velocity>) -> Result<(), StepperBuilderError> {
                if let Some(velocity) = velocity_opt {
                    if velocity.is_normal() {
                        self._velocity_max = Some(velocity.abs()); 
                        self.update_start_stop()
                    } else {
                        Err(StepperBuilderError::InvalidVelocity(velocity))
                    }
                } else {
                    self._velocity_max = None;
                    Ok(())
                }
            }
        //

        // Acceleration
            fn acceleration_max(&self) -> Option<Acceleration> {
                self._acceleration_max   
            }

            fn set_acceleration_max(&mut self, acceleration_opt : Option<Acceleration>) -> Result<(), StepperBuilderError> {
                if let Some(acceleration) = acceleration_opt {
                    if acceleration.is_normal() {
                        self._acceleration_max = Some(acceleration.abs()); 
                        self.update_start_stop()
                    } else {
                        Err(StepperBuilderError::InvalidAcceleration(acceleration))
                    }
                } else {
                    self._acceleration_max = None;
                    Ok(())
                }
            }
        // 
        
        #[inline]
        fn drive_mode(&self) -> &DriveMode {
            &self.mode
        }

        fn set_drive_mode<C : StepperController>(&mut self, mode : DriveMode, ctrl : &mut C) -> Result<(), StepperBuilderError> {
            match mode {
                // Driving with a constant velocity
                DriveMode::ConstVelocity(mut velocity, dir) => {
                    velocity = velocity.abs();

                    if velocity > self.velocity_possible() {
                        return Err(StepperBuilderError::VelocityTooHigh(velocity, self.velocity_possible()))
                    } 

                    self._direction = dir;
                    ctrl.set_dir(dir)?;
                },
                // Drive with a constant factor
                DriveMode::ConstFactor(_, dir) => {
                    self._direction = dir;
                    ctrl.set_dir(dir)?;
                },
                DriveMode::FixedDistance(rel_dist, velocity_exit, _) => {
                    if velocity_exit > self.velocity_possible() {
                        return Err(StepperBuilderError::VelocityTooHigh(velocity_exit, self.velocity_possible()))
                    }

                    self.distance = self._consts.steps_from_angle_abs(rel_dist, self._microsteps);
                    self.distance_counter = 0;

                    if rel_dist >= RelDist::ZERO {
                        self._direction = Direction::CW;
                    } else {
                        self._direction = Direction::CCW;
                    }
                    ctrl.set_dir(self._direction)?;
                },
                _ => { }
            };

            self.mode = mode;
            Ok(())
        }
    }

    impl DefinedActuator for StartStopBuilder {
        fn ptp_time_for_distance(&self, abs_pos_0 : AbsPos, abs_pos_t : AbsPos) -> Time {
            (abs_pos_t - abs_pos_0) / self.velocity_possible()
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
        _step_angle : RelDist, 
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
        pub fn update(&mut self) -> Result<(), StepperBuilderError> {
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
                let accel = self.consts().acceleration_max_for_velocity(self.vars(), self.config(), vel, self.direction())
                    .ok_or(StepperBuilderError::Overload)?;
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
        pub fn goto_velocity(&mut self, vel_tar : Velocity) -> Result<Velocity, StepperBuilderError> {
            let vel_below = self.current_speed_level.checked_sub(2)
                .map(|i| self.speed_levels[i])
                .unwrap_or(Velocity::ZERO);

            if vel_tar > self.velocity_current() {
                // Desired velocity is greater than the current speed, increasing speed level if possible
                if let Some(&time) = self.times.get(self.current_speed_level) {
                    self.current_speed_level += 1;
                    Ok(self.consts().velocity(time, self.microsteps()))
                } else {
                    Err(StepperBuilderError::VelocityTooHigh(vel_tar, *self.speed_levels.last().unwrap_or(&Velocity::ZERO)))
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
                DriveMode::ConstFactor(factor, _) => self.goto_velocity(self.velocity_possible() * factor).ok(),
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
                        self.goto_velocity(self.velocity_possible() * factor).ok()
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
            fn new(consts : StepperConst, config : StepperConfig) -> Result<Self, StepperBuilderError>
            where 
                Self: Sized 
            {
                let mut _self = Self {
                    _vars: ActuatorVars::ZERO,
                    _config: config,

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

            fn step_angle(&self) -> RelDist {
                self._step_angle
            }

            fn direction(&self) -> Direction {
                self._dir
            }
        //

        // Setters
            fn set_microsteps(&mut self, microsteps : MicroSteps) -> Result<(), StepperBuilderError> {
                self._step_angle = self._consts.step_angle(microsteps);
                self._microsteps = microsteps;
                self.update()
            }

            fn set_config(&mut self, config : StepperConfig) -> Result<(), StepperBuilderError> {
                self._config = config;
                self.update() 
            }

            fn set_overload_curret(&mut self, current : Option<f32>) -> Result<(), StepperBuilderError> {
                self._config.overload_current = current;
                self.update()
            }
        // 

        // Loads
            fn apply_gen_force(&mut self, force : Force) -> Result<(), StepperBuilderError> {
                self._vars.force_load_gen = force;
                self.update()
            }

            fn apply_dir_force(&mut self, force : Force) -> Result<(), StepperBuilderError> {
                self._vars.force_load_dir = force;
                self.update()
            }
            
            fn apply_inertia(&mut self, inertia : Inertia) -> Result<(), StepperBuilderError> {
                self._vars.inertia_load = inertia;
                self.update()
            }
        // 

        fn velocity_possible(&self) -> Velocity {
            self.velocity_cap().min(
                *self.speed_levels.last().unwrap_or(&Velocity::ZERO)
            )
        }

        fn set_velocity_max(&mut self, velocity_cap : Velocity) -> Result<(), StepperBuilderError> {
            if velocity_cap.is_normal() {
                self._velocity_cap = Some(velocity_cap.abs()); 
                self.update()
            } else {
                Err(StepperBuilderError::InvalidVelocity(velocity_cap))
            }
        }

        fn drive_mode(&self) -> &DriveMode {
            &self.mode
        }

        fn set_drive_mode<C : StepperController>(&mut self, mode : DriveMode, ctrl : &mut C) -> Result<(), StepperBuilderError> {
            match mode {
                DriveMode::ConstVelocity(velocity, dir) => {
                    if velocity > self.velocity_possible() {
                        return Err(StepperBuilderError::VelocityTooHigh(velocity, self.velocity_possible()))
                    } 

                    if self.mode != DriveMode::Inactive {
                        // Turn around motor
                        self.stop_with_mode(mode.clone());
                    } else {
                        self._dir = dir;
                        ctrl.set_dir(dir)?;
                    }
                },
                DriveMode::ConstFactor(_, dir) => {
                    if (self.mode != DriveMode::Inactive) & (dir != self._dir) {
                        // Turn around motor
                        self.stop_with_mode(mode.clone());
                    } else {
                        self._dir = dir;
                        ctrl.set_dir(dir)?;
                    }
                },
                DriveMode::FixedDistance(rel_dist, velocity_exit, _) => {
                    if velocity_exit > self.velocity_possible() {
                        return Err(StepperBuilderError::VelocityTooHigh(velocity_exit, self.velocity_possible()))
                    }

                    self.distance = self._consts.steps_from_angle_abs(rel_dist, self._microsteps);
                    self.distance_counter = 0;

                    if self.distance < self.current_speed_level as u64 {
                        return Err(StepperBuilderError::DistanceTooShort(self.step_angle(), self.distance, self.current_speed_level as u64))
                    }

                    if rel_dist >= RelDist::ZERO {
                        self._dir = Direction::CW;
                    } else {
                        self._dir = Direction::CCW;
                    }
                    ctrl.set_dir(self._dir)?;
                },
                _ => { }
            };

            self.mode = mode;
            Ok(())
        }
    }
    
    impl DefinedActuator for ComplexBuilder {
        fn ptp_time_for_distance(&self, abs_pos_0 : AbsPos, abs_pos_t : AbsPos) -> Time {
            let rel_dist = abs_pos_t - abs_pos_0;
            let distance = self.consts().steps_from_angle_abs(rel_dist, self.microsteps());

            let max_speed_level = (distance / 2).saturating_sub(1);

            // Multiple cases
            if distance == 1 {
                *self.times.first().unwrap_or(&Time::INFINITY)
            } else if max_speed_level < self.speed_levels.len() as u64 {
                self.time_sums[max_speed_level as usize] * 2.0
                    + self.consts().step_time(self.speed_levels[max_speed_level as usize], self.microsteps())
            } else {
                let distance_rest = distance - self.times.len() as u64 * 2;

                self.consts().step_time(self.velocity_possible(), self.microsteps()) * distance_rest as f32
                    + *self.time_sums.last().unwrap_or(&Time::ZERO) * 2.0
            }
        }
    }
// 