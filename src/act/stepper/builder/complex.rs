use alloc::vec::Vec;

use syunit::*;

use crate::act::stepper::builder::StepperBuilderAdvanced;
use crate::{StepperConst, StepperConfig};
use crate::act::stepper::StepperController;
use crate::data::{ActuatorVars, MicroSteps};
use crate::math;
use crate::math::movements::DefinedActuator;

use super::{DriveMode, StepperBuilder, ActuatorError, DEFAULT_MAX_SPEED_LEVEL};

/// ########################
/// #    ComplexBuilder    #
/// ########################
/// 
/// A more complex builder that introduces speed levels 
/// 
/// - High movements speeds
/// - Perfect for microstepping
/// 
#[derive(Debug)]
pub struct ComplexBuilder {
    _consts : StepperConst,
    _vars : ActuatorVars,
    _config : StepperConfig,

    // Limits
    _velocity_max : Option<Velocity>,
    _acceleration_max : Option<Acceleration>,
    _jolt_max : Option<Jolt>,

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
    pub fn update(&mut self) -> Result<(), ActuatorError> {
        // Store relevant values
        let max_speed_level = self.max_speed_level.unwrap_or(DEFAULT_MAX_SPEED_LEVEL);
        let velocity_cap = self.velocity_cap();

        // Create new arrays
        let mut speed_levels : Vec<Velocity> = Vec::new();
        let mut time_sums : Vec<Time> = Vec::new();
        let mut times : Vec<Time> = Vec::new();

        let mut vel = Velocity::ZERO;

        // Iterate to max speed level or until the cap is reached
        for _ in 0 .. max_speed_level {
            // Calculate acceleration and movement time when fully accelerating
            let accel = self.consts().acceleration_max_for_velocity(self.vars(), self.config(), vel, self.direction())
                .ok_or(ActuatorError::Overload)?;
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
    pub fn goto_velocity(&mut self, vel_tar : Velocity) -> Result<Velocity, ActuatorError> {
        let vel_below = self.current_speed_level.checked_sub(2)
            .map(|i| self.speed_levels[i])
            .unwrap_or(Velocity::ZERO);

        if vel_tar > self.velocity_current() {
            // Desired velocity is greater than the current speed, increasing speed level if possible
            if let Some(&time) = self.times.get(self.current_speed_level) {
                self.current_speed_level += 1;
                Ok(self.consts().velocity(time, self.microsteps()))
            } else {
                Err(ActuatorError::VelocityTooHigh(vel_tar, *self.speed_levels.last().unwrap_or(&Velocity::ZERO)))
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

    // Velocity
        /// Returns the cap velocity
        /// - Is either the cap velocity given by the user
        /// - Or the maximum recommended velocity for a stepper motor
        /// depends on which is lower
        pub fn velocity_cap(&self) -> Velocity {
            self._velocity_max.unwrap_or(Velocity::INFINITY)
                .min(self.consts().velocity_max(self.config().voltage))
        }

        /// The maximum velocity that is currently possible, defined by numerous factors like maximum jolt, acceleration, velocity and start-stop mechanics
        pub fn velocity_possible(&self) -> Velocity {
            self.velocity_cap().min(
                *self.speed_levels.last().unwrap_or(&Velocity::ZERO)
            )
        }
    // 
}

impl Iterator for ComplexBuilder {
    type Item = Time;

    fn next(&mut self) -> Option<Self::Item> {
        let mut vel_opt = match self.mode {
            DriveMode::ConstVelocity(velocity ) => self.goto_velocity(velocity).ok(),
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
    // Getters
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
        fn set_microsteps(&mut self, microsteps : MicroSteps) -> Result<(), ActuatorError> {
            self._step_angle = self._consts.step_angle(microsteps);
            self._microsteps = microsteps;
            self.update()
        }

        fn set_overload_curret(&mut self, current : Option<f32>) -> Result<(), ActuatorError> {
            self._config.overload_current = current;
            self.update()
        }
    // 

    // Velocity
        #[inline]
        fn velocity_max(&self) -> Option<Velocity> {
            self._velocity_max
        }

        fn set_velocity_max(&mut self, velocity_opt : Option<Velocity>) -> Result<(), ActuatorError> {
            if let Some(velocity) = velocity_opt {
                if velocity.is_normal() {
                    self._velocity_max = Some(velocity.abs()); 
                    self.update()
                } else {
                    Err(ActuatorError::InvalidVelocity(velocity))
                }
            } else {
                self._velocity_max = None;
                Ok(())
            }
        }
    //

    // Acceleration
        #[inline]
        fn acceleration_max(&self) -> Option<Acceleration> {
            self._acceleration_max   
        }

        fn set_acceleration_max(&mut self, acceleration_opt : Option<Acceleration>) -> Result<(), ActuatorError> {
            if let Some(acceleration) = acceleration_opt {
                if acceleration.is_normal() {
                    self._acceleration_max = Some(acceleration.abs()); 
                    self.update()
                } else {
                    Err(ActuatorError::InvalidAcceleration(acceleration))
                }
            } else {
                self._acceleration_max = None;
                Ok(())
            }
        }
    // 

    // Jolt 
        #[inline]
        fn jolt_max(&self) -> Option<Jolt> {
            self._jolt_max
        }

        fn set_jolt_max(&mut self, jolt_opt : Option<Jolt>) -> Result<(), ActuatorError> {
            if let Some(jolt) = jolt_opt {
                if jolt.is_normal() {
                    self._jolt_max = Some(jolt.abs()); 
                    self.update()
                } else {
                    Err(ActuatorError::InvalidJolt(jolt))
                }
            } else {
                self._jolt_max = None;
                Ok(())
            }
        }
    // 

    fn drive_mode(&self) -> &DriveMode {
        &self.mode
    }

    fn set_drive_mode<C : StepperController>(&mut self, mode : DriveMode, ctrl : &mut C) -> Result<(), ActuatorError> {
        match mode {
            DriveMode::ConstVelocity(mut velocity) => {
                let dir = velocity.get_direction();
                velocity = velocity.abs();

                if velocity > self.velocity_possible() {
                    return Err(ActuatorError::VelocityTooHigh(velocity, self.velocity_possible()))
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
                    return Err(ActuatorError::VelocityTooHigh(velocity_exit, self.velocity_possible()))
                }

                self.distance = self._consts.steps_from_angle_abs(rel_dist, self._microsteps);
                self.distance_counter = 0;

                if self.distance < self.current_speed_level as u64 {
                    return Err(ActuatorError::InvaldRelativeDistance(self.step_angle()))
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

impl StepperBuilderAdvanced for ComplexBuilder {
    // General constructors
        fn new(consts : StepperConst, config : StepperConfig) -> Result<Self, ActuatorError>
        where 
            Self: Sized 
        {
            let mut _self = Self {
                // Data
                _vars: ActuatorVars::ZERO,
                _config: config,

                // Limits
                _velocity_max: None,
                _acceleration_max: None,
                _jolt_max: None,

                _microsteps: MicroSteps::default(),

                distance: 0,
                distance_counter: 0,
                _step_angle: consts.step_angle(MicroSteps::default()),
                _dir: Direction::default(),

                mode: DriveMode::Inactive,
                cached_mode: None,

                time_sums: Vec::new(),
                times: Vec::new(),
                speed_levels: Vec::new(),
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
    //

    // Setters
        fn set_config(&mut self, config : StepperConfig) -> Result<(), ActuatorError> {
            self._config = config;
            self.update() 
        }
    // 

    // Loads
        fn apply_gen_force(&mut self, force : Force) -> Result<(), ActuatorError> {
            self._vars.force_load_gen = force;
            self.update()
        }

        fn apply_dir_force(&mut self, force : Force) -> Result<(), ActuatorError> {
            self._vars.force_load_dir = force;
            self.update()
        }
        
        fn apply_inertia(&mut self, inertia : Inertia) -> Result<(), ActuatorError> {
            self._vars.inertia_load = inertia;
            self.update()
        }
    // 
}

// Math implementations
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