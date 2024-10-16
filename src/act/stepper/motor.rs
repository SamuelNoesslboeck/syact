use alloc::boxed::Box;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::sync::atomic::Ordering::Relaxed;

use syunit::*;

use crate::{AsyncActuator, SyncActuator, SyncActuatorBlocking};
use crate::act::{ActuatorError, InterruptReason, Interruptible, Interruptor, SyncActuatorAdvanced, SyncActuatorState};
use crate::act::asyn::AsyncActuatorState;
use crate::act::stepper::{StepperActuator, StepperController, StepperBuilder, DriveMode, StepperState};
use crate::act::stepper::builder::{StepperBuilderAdvanced, StepperBuilderSimple};
use crate::data::{StepperConfig, StepperConst, MicroSteps}; 
use crate::math::movements::DefinedActuator;

/// A stepper motor
/// 
/// Controlled by two pins, one giving information about the direction, the other about the step signal (PWM)
pub struct StepperMotor<B : StepperBuilder, C : StepperController> {
    builder : B,
    ctrl : C, 

    _state: Arc<StepperState>,

    // Limits
    _limit_min : Option<AbsPos>,
    _limit_max : Option<AbsPos>,

    // Interrupters
    interruptors : Vec<Box<dyn Interruptor + Send>>,
    _intr_reason : Option<InterruptReason>,
}

// Inits
impl<B : StepperBuilder, C : StepperController> StepperMotor<B, C> {   
    /// ######################################
    /// #    StepperMotor::handle_builder    #
    /// ######################################
    ///
    /// Main driving algorithm for stepper motors, handles the builder until no nodes are left anymore
    /// 
    /// ## Thread
    /// 
    /// Blocks the current thread and creates the step signals until the builder is finished
    pub fn handle_builder(&mut self) -> Result<(), ActuatorError> {
        // Update the movement variable of the state
        self._state._moving.store(false, Relaxed);
        
        // Iterate through the builder until no nodes are left
        while let Some(node) = self.builder.next() {
            // Get the current direction of the motor (builder)
            let direction = self.builder.direction();
            // Get the current drive mode of the motor (builder)
            let drive_mode = self.builder.drive_mode();

            // Check all interruptors if the motor is not stopping already
            if *drive_mode != DriveMode::Stop {
                for intr in self.interruptors.iter_mut() {
                    // Check if the direction is right
                    if let Some(i_dir) = intr.dir() {
                        if i_dir != direction {
                            // If the interruptors checking-direction does not match the current direction, 
                            //     the loop skips to the next interruptor
                            continue;
                        }
                    }

                    // Checks if the interruptor has been triggered
                    if let Some(reason) = intr.check(self._state.abs_pos()) {
                        intr.set_temp_dir(Some(direction));
                        self._intr_reason.replace(reason);
                        
                        self.builder.set_drive_mode(DriveMode::Stop, &mut self.ctrl)?; 
                    } else {
                        // Clear temporary direction
                        intr.set_temp_dir(None);
                    }
                }
            }

            // Make step and return error if occured
            self.ctrl.step(node)?;

            // Check if the abs_pos value exeeds any limits, stop the movement if it does
            if direction.as_bool() {
                self._state._abs_pos.fetch_add(self.builder.step_angle().0, Relaxed);

                if self.abs_pos() > self.limit_max().unwrap_or(AbsPos::INFINITY) {
                    self.builder.set_drive_mode(DriveMode::Stop, &mut self.ctrl)?;
                } 
            } else {
                self._state._abs_pos.fetch_sub(self.builder.step_angle().0, Relaxed);

                if self.abs_pos() < self.limit_min().unwrap_or(AbsPos::NEG_INFINITY) {
                    self.builder.set_drive_mode(DriveMode::Stop, &mut self.ctrl)?;
                } 
            }
        }

        // No movement anymore
        self._state._moving.store(false, Relaxed);

        Ok(())
    }

    /// Returns the current movement direction
    pub fn direction(&self) -> Direction {
        self.builder.direction()
    }
}

// #######################################
// #    SyncActuator - Implementation    #
// #######################################
    impl<B : StepperBuilder, C : StepperController> SyncActuator for StepperMotor<B, C> {
        // State
            fn state(&self) -> &dyn SyncActuatorState {
                self._state.as_ref()
            }

            fn clone_state(&self) -> Arc<dyn SyncActuatorState> {
                self._state.clone()
            }
        // 

        // Position 
            #[inline]
            fn abs_pos(&self) -> AbsPos {
                self._state.abs_pos()
            }   

            #[inline]
            fn overwrite_abs_pos(&mut self, abs_pos : AbsPos) {
                self._state._abs_pos.store(abs_pos.0, Relaxed);
            }
        //

        // Velocity
            #[inline]
            fn velocity_max(&self) -> Option<Velocity> {
                self.builder.velocity_max()
            }

            #[inline]
            fn set_velocity_max(&mut self, velocity_opt : Option<Velocity>) -> Result<(), ActuatorError> {
                self.builder.set_velocity_max(velocity_opt)?;
                Ok(())
            }
        //

        // Acceleration
            #[inline]
            fn acceleration_max(&self) -> Option<Acceleration> {
                self.builder.acceleration_max()
            }

            fn set_acceleration_max(&mut self, acceleration_opt : Option<Acceleration>) -> Result<(), ActuatorError> {
                self.builder.set_acceleration_max(acceleration_opt)?;
                Ok(())
            }
        //

        // Jolt
            fn jolt_max(&self) -> Option<Jolt> {
                self.builder.jolt_max()
            }

            fn set_jolt_max(&mut self, jolt_opt : Option<Jolt>) -> Result<(), ActuatorError> {
                self.builder.set_jolt_max(jolt_opt)?;
                Ok(())
            }
        //

        // Position limits
            #[inline]
            fn limit_max(&self) -> Option<AbsPos> {
                self._limit_max
            }

            #[inline]
            fn limit_min(&self) -> Option<AbsPos> {
                self._limit_min
            }
            
            #[inline]
            fn set_pos_limits(&mut self, min : Option<AbsPos>, max : Option<AbsPos>) {
                if let Some(min) = min {
                    self._limit_min = Some(min)
                }

                if let Some(max) = max {
                    self._limit_max = Some(max);
                }
            }

            #[inline]
            fn overwrite_pos_limits(&mut self, min : Option<AbsPos>, max : Option<AbsPos>) {
                self._limit_min = min;
                self._limit_max = max;
            }

            fn resolve_pos_limits_for_abs_pos(&self, abs_pos : AbsPos) -> RelDist {
                if let Some(ang) = self.limit_min() {
                    if abs_pos < ang {
                        abs_pos - ang
                    } else {
                        if let Some(ang) = self.limit_max() {
                            if abs_pos > ang {
                                abs_pos - ang
                            } else { 
                                RelDist::ZERO 
                            }
                        } else {
                            RelDist::ZERO
                        }
                    }
                } else {
                    if let Some(ang) = self.limit_max() {
                        if abs_pos > ang {
                            abs_pos - ang
                        } else { 
                            RelDist::ZERO 
                        }
                    } else {
                        RelDist::NAN
                    }
                }
            }

            fn set_endpos(&mut self, overwrite_abs_pos : AbsPos) {
                self.overwrite_abs_pos(overwrite_abs_pos);

                let dir = self.direction().as_bool();
        
                self.set_pos_limits(
                    if dir { None } else { Some(overwrite_abs_pos) },
                    if dir { Some(overwrite_abs_pos) } else { None }
                )
            }
        //
    }

    impl<B : StepperBuilder, C : StepperController> SyncActuatorBlocking for StepperMotor<B, C> {
        fn drive_rel(&mut self, rel_dist : RelDist, speed_f : Factor) -> Result<(), ActuatorError> {
            if !rel_dist.is_finite() {
                return Err(ActuatorError::InvaldRelativeDistance(rel_dist));
            }

            // Set drive mode, return mapped error if one occurs
            self.builder.set_drive_mode(DriveMode::FixedDistance(rel_dist, Velocity::ZERO, speed_f), &mut self.ctrl)?;
            self.handle_builder()
        }
    }
// 

// ###########################
// #    Builder dependent    #
// ###########################
    impl<B : StepperBuilderSimple, C : StepperController> StepperMotor<B, C> {
        /// Creates a new stepper motor with the given controller `ctrl` 
        pub fn new_simple(ctrl : C) -> Result<Self, ActuatorError> {
            Ok(Self {
                builder: B::new()?,
                ctrl,

                _state : Arc::new(StepperState::new()),

                _limit_min: None,
                _limit_max: None,

                interruptors : Vec::new(),
                _intr_reason: None
            })
        }
    }

    impl<B : StepperBuilderAdvanced, C : StepperController> StepperMotor<B, C> {
        /// Creates a new stepper motor with the given constants `consts` and configuration `config`
        pub fn new_advanced(ctrl : C, consts : StepperConst, config : StepperConfig) -> Result<Self, ActuatorError> {
            Ok(Self {
                builder: B::new(consts, config)?,
                ctrl,

                _state : Arc::new(StepperState::new()),

                _limit_min: None,
                _limit_max: None,

                interruptors : Vec::new(),
                _intr_reason: None
            })
        }
    }

    impl<B : StepperBuilderAdvanced, C : StepperController> SyncActuatorAdvanced for StepperMotor<B, C> {
        // Loads
            fn force_gen(&self) -> Force {
                self.builder.vars().force_load_gen
            }

            fn force_dir(&self) -> Force {
                self.builder.vars().force_load_dir
            }

            fn apply_gen_force(&mut self, force : Force) -> Result<(), ActuatorError> {
                self.builder.apply_gen_force(force)?;
                Ok(())
            }

            fn apply_dir_force(&mut self, force : Force) -> Result<(), ActuatorError> {
                self.builder.apply_dir_force(force)?;
                Ok(())
            }

            fn inertia(&self) -> Inertia {
                self.builder.vars().inertia_load
            }

            #[inline(always)]
            fn apply_inertia(&mut self, inertia : Inertia) -> Result<(), ActuatorError> {
                self.builder.apply_inertia(inertia)?;
                Ok(())
            }
        //
    }
// 

impl<B : StepperBuilder, C : StepperController> StepperActuator for StepperMotor<B, C> 
where
    B : DefinedActuator 
{
    // Data
        fn microsteps(&self) -> MicroSteps {
            self.builder.microsteps()
        }

        fn set_microsteps(&mut self, microsteps : MicroSteps) -> Result<(), ActuatorError> {
            self.builder.set_microsteps(microsteps)
            
        }
    //

    fn step_ang(&self) -> RelDist {
        self.builder.step_angle()
    }
}

impl<B : StepperBuilder, C : StepperController> Interruptible for StepperMotor<B, C> {
    // Interruptors
        fn add_interruptor(&mut self, interruptor : Box<dyn Interruptor + Send>) {
            self.interruptors.push(interruptor);
        }

        fn intr_reason(&mut self) -> Option<InterruptReason> {
            // Return the value and replace it with `None`
            core::mem::replace(&mut self._intr_reason, None)
        }
    // 
}

impl<B : StepperBuilder, C : StepperController> DefinedActuator for StepperMotor<B, C> 
where
    B : DefinedActuator 
{
    fn ptp_time_for_distance(&self, abs_pos_0 : AbsPos, abs_pos_t : AbsPos) -> Time {
        self.builder.ptp_time_for_distance(abs_pos_0, abs_pos_t)
    }
}

impl<B : StepperBuilder, C : StepperController> AsyncActuator for StepperMotor<B, C> 
{
    fn drive_factor(&mut self, speed : Factor, direction : Direction) -> Result<(), ActuatorError> {
        // Set drive mode, return mapped error if one occurs
        self.builder.set_drive_mode(DriveMode::ConstFactor(speed, direction), &mut self.ctrl)?;
        self.handle_builder()
    }

    fn drive_speed(&mut self, speed : Velocity) -> Result<(), ActuatorError> {
        // Set drive mode, return mapped error if one occurs
        self.builder.set_drive_mode(DriveMode::ConstVelocity(speed), &mut self.ctrl)?;
        self.handle_builder()
    }

    // State
        fn state(&self) -> &dyn AsyncActuatorState {
            self._state.as_ref()
        }

        fn clone_state(&self) -> Arc<dyn AsyncActuatorState> {
            self._state.clone()
        }
    // 
}