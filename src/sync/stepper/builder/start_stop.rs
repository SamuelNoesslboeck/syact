use syunit::*;
use syunit::metric::*;

use crate::{StepperConst, StepperConfig, DefinedActuator};
use crate::sync::stepper::StepperController;
use crate::sync::stepper::builder::AdvancedStepperBuilder;
use crate::data::{ActuatorVars, MicroSteps};

use super::{DriveMode, StepperBuilder, ActuatorError};


/// ##########################
/// #    StartStopBuilder    #
/// ##########################
/// 
/// A simple builder that moves the stepper motor only in its start-stop-range  
/// 
/// - High performance
/// - Very safe
/// - Low movement speed
/// - Microstepping basically impossible
/// 
#[derive(Debug)]
pub struct StartStopBuilder {
    // Data
    _consts : StepperConst,
    _vars : ActuatorVars,
    _config : StepperConfig,

    /// Start-Stop speed
    velocity_start_stop : RadPerSecond,

    // Limits
    _velocity_max : Option<RadPerSecond>,
    _acceleration_max : Option<RadPerSecond2>,
    _jolt_max : Option<RadPerSecond3>,

    _microsteps : MicroSteps,   
    _step_angle : Radians, 
    _direction : Direction,
    mode : DriveMode,

    // Step counters
    distance : u64,
    distance_counter : u64
}

impl StartStopBuilder {
    /// Updates the builders velocity values considering the loads etc.
    pub fn update_start_stop(&mut self) -> Result<(), ActuatorError> {
        self.velocity_start_stop = self.consts().velocity_start_stop(self.vars(), self.config(), self._microsteps)
            .ok_or(ActuatorError::Overload)?;

        Ok(())
    }

    // RadPerSecond2 helper
        /// Returns the maximum acceleration that can be reached when using the maximum jolt specified
        /// 
        /// ### Option
        /// 
        /// Returns `None` if no maximum jolt is defined
        pub fn acceleration_by_max_jolt(&self) -> Option<RadPerSecond2> {
            self.jolt_max().map(|jolt_max| {
                sykin::kin3::acceleration_for_distance_only_jolt::<Rotary>(self.step_angle(), jolt_max)
            })
        }

        /// Returns the maximum allowed acceleration, returns `RadPerSecond2::INFINITY` if no maximum acceleration nor a maximum jolt has been specified
        pub fn acceleration_allowed(&self) -> RadPerSecond2 {
            self.acceleration_max().unwrap_or(RadPerSecond2::INFINITY)
                .min(self.acceleration_by_max_jolt().unwrap_or(RadPerSecond2::INFINITY))
        }
    // 

    // RadPerSecond helpers
        /// The maximum velocity that can be reached with the specified acceleration (will result in infinity if no limits are set)
        pub fn velocity_by_max_acceleration(&self) -> RadPerSecond {
            sykin::kin2::velocity_for_distance_no_vel0::<Rotary>(self.step_angle(), self.acceleration_allowed())
        }

        /// The maximum velocity that is currently possible, defined by numerous factors like maximum jolt, acceleration, velocity and start-stop mechanics
        pub fn velocity_possible(&self) -> RadPerSecond {
            self.velocity_start_stop.min(
                self._velocity_max.unwrap_or(RadPerSecond::INFINITY)
            ).min(
                self.velocity_by_max_acceleration()
            )
        }
    //
}

// The iterator yields the time values for the stepper motor
impl Iterator for StartStopBuilder {
    type Item = Seconds;

    fn next(&mut self) -> Option<Self::Item> {
        match self.mode {
            // Contant velocity just yields the velocity, the check if the velocity is possible already happened in `set_drive_mode()`
            DriveMode::ConstVelocity(velocity ) => Some(velocity),
            // Drive a constant factor of the maximum possible velocity
            DriveMode::ConstFactor(factor, _) => Some(self.velocity_possible() * factor),
            // Continue driving until the distance is reached
            DriveMode::FixedDistance(_, _, factor) => {
                self.distance_counter += 1;

                if self.distance_counter > self.distance {
                    self.mode = DriveMode::Inactive;
                    None
                } else {
                    Some(self.velocity_possible() * factor)
                }
            },
            // Stop instantly, that is possible thanks to the start-stop procedure being used
            DriveMode::Stop => {
                self.mode = DriveMode::Inactive;
                None
            },
            // Inactive, no more nodes needed
            DriveMode::Inactive => None
        }.map(|velocity | self._consts.step_time(velocity , self._microsteps))
    }
}

impl StepperBuilder for StartStopBuilder {

    // Data
        fn microsteps(&self) -> MicroSteps {
            self._microsteps
        }

        fn set_microsteps(&mut self, microsteps : MicroSteps) -> Result<(), ActuatorError> {
            // Update step-angle when changing microsteps
            self._step_angle = self._consts.step_angle(microsteps);
            self._microsteps = microsteps;
            self.update_start_stop()        // Microsteps affect start stop velocity, recalculate
        }

        fn step_angle(&self) -> Radians {
            self._step_angle
        }

        fn direction(&self) -> Direction {
            self._direction
        }
    // 

    // RadPerSecond
        #[inline]
        fn velocity_max(&self) -> Option<RadPerSecond> {
            self._velocity_max
        }

        fn set_velocity_max(&mut self, velocity_opt : Option<RadPerSecond>) -> Result<(), ActuatorError> {
            if let Some(velocity) = velocity_opt {
                if velocity.is_normal() {
                    self._velocity_max = Some(velocity.abs()); 
                    self.update_start_stop()
                } else {
                    Err(ActuatorError::InvalidVelocity(velocity))
                }
            } else {
                self._velocity_max = None;
                Ok(())
            }
        }
    //

    // RadPerSecond2
        #[inline]
        fn acceleration_max(&self) -> Option<RadPerSecond2> {
            self._acceleration_max   
        }

        fn set_acceleration_max(&mut self, acceleration_opt : Option<RadPerSecond2>) -> Result<(), ActuatorError> {
            if let Some(acceleration) = acceleration_opt {
                if acceleration.is_normal() {
                    self._acceleration_max = Some(acceleration.abs()); 
                    self.update_start_stop()
                } else {
                    Err(ActuatorError::InvalidAcceleration(acceleration))
                }
            } else {
                self._acceleration_max = None;
                Ok(())
            }
        }
    // 

    // RadPerSecond3 
        #[inline]
        fn jolt_max(&self) -> Option<RadPerSecond3> {
            self._jolt_max
        }

        fn set_jolt_max(&mut self, jolt_opt : Option<RadPerSecond3>) -> Result<(), ActuatorError> {
            if let Some(jolt) = jolt_opt {
                if jolt.is_normal() {
                    self._jolt_max = Some(jolt.abs()); 
                    self.update_start_stop()
                } else {
                    Err(ActuatorError::InvalidJolt(jolt))
                }
            } else {
                self._jolt_max = None;
                Ok(())
            }
        }
    // 
    
    #[inline]
    fn drive_mode(&self) -> &DriveMode {
        &self.mode
    }

    fn set_drive_mode<C : StepperController>(&mut self, mode : DriveMode, ctrl : &mut C) -> Result<(), ActuatorError> {
        match mode {
            // Driving with a constant velocity, check if the velocity is possible, return error if it is not
            DriveMode::ConstVelocity(mut velocity) => {
                let dir = velocity.get_direction();
                velocity = velocity.abs();

                if velocity > self.velocity_possible() {
                    return Err(ActuatorError::VelocityTooHigh(velocity, self.velocity_possible()))
                } 

                self._direction = dir;
                ctrl.set_dir(dir)?;
            },
            // Drive with a constant factor, always possible
            DriveMode::ConstFactor(_, dir) => {
                self._direction = dir;
                ctrl.set_dir(dir)?;
            },
            // Check if the exit velocity is possible, everything else is fine
            DriveMode::FixedDistance(rel_dist, velocity_exit, _) => {
                if velocity_exit > self.velocity_possible() {
                    return Err(ActuatorError::VelocityTooHigh(velocity_exit, self.velocity_possible()))
                }

                self.distance = self._consts.steps_from_angle_abs(rel_dist, self._microsteps);
                self.distance_counter = 0;

                if rel_dist >= Radians::ZERO {
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

// Extension traits
    impl AdvancedStepperBuilder for StartStopBuilder {
        // General constructors
            fn new(consts : StepperConst, config : StepperConfig) -> Result<Self, ActuatorError>
            where 
                Self: Sized 
            {
                let mut _self = Self {
                    _vars: ActuatorVars::ZERO,
                    _config: config,
    
                    velocity_start_stop: RadPerSecond::INFINITY,
    
                    _velocity_max: None,
                    _acceleration_max: None,
                    _jolt_max: None,
    
                    _step_angle: consts.step_angle(MicroSteps::default()),
                    _direction: Direction::default(),
                    _microsteps: MicroSteps::default(),
                    mode: DriveMode::Inactive,
    
                    distance: 0,
                    distance_counter: 0,
    
                    _consts: consts
                };
    
                _self.update_start_stop()?;
    
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
                self.update_start_stop()
            }


            fn set_overload_curret(&mut self, current : Option<f32>) -> Result<(), ActuatorError> {
                self._config.overload_current = current;
                self.update_start_stop()        // Overload current affects start stop velocity, recalculate
            }
        //

        // Loads
            fn apply_gen_force(&mut self, force : NewtonMeters) -> Result<(), ActuatorError> {
                self._vars.force_load_gen = force;
                self.update_start_stop()
            }
    
            fn apply_dir_force(&mut self, force : NewtonMeters) -> Result<(), ActuatorError> {
                self._vars.force_load_dir = force;
                self.update_start_stop()
            }
            
            fn apply_inertia(&mut self, inertia : KgMeter2) -> Result<(), ActuatorError> {
                self._vars.inertia_load = inertia;
                self.update_start_stop()
            }
        // 
    }
// 

// Math
    impl DefinedActuator for StartStopBuilder {
        fn ptp_time_for_distance(&self, abs_pos_0 : PositionRad, abs_pos_t : PositionRad) -> Seconds {
            (abs_pos_t - abs_pos_0) / self.velocity_possible()
        }
    }
// 