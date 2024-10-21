use core::sync::atomic::Ordering::Relaxed;

use alloc::boxed::Box;
use alloc::sync::Arc;
use alloc::vec::Vec;

use syunit::*;
use syunit::metric::*;

use crate::{SyncActuator, SyncActuatorBlocking, InterruptReason, Interruptible, Interruptor, AdvancedActuator, DefinedActuator};
use crate::data::{StepperConfig, StepperConst, MicroSteps}; 
use crate::sync::{ActuatorError, SyncActuatorState};
use crate::sync::stepper::{StepperActuator, StepperController, StepperBuilder, DriveMode, StepperState};
use crate::sync::stepper::builder::{StepperBuilderAdvanced, StepperBuilderSimple};

/// A stepper motor
/// 
/// Controlled by two pins, one giving information about the direction, the other about the step signal (PWM)
pub struct StepperMotor<B : StepperBuilder, C : StepperController> {
    builder : B,
    ctrl : C, 

    _state: Arc<StepperState>,

    // Limits
    _limit_min : Option<PositionRad>,
    _limit_max : Option<PositionRad>,

    // Interrupters
    interruptors : Vec<Box<dyn Interruptor<Rotary> + Send>>,
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
                    if let Some(reason) = intr.check(self._state.pos()) {
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

            // Check if the pos value exeeds any limits, stop the movement if it does
            if direction.as_bool() {
                self._state._abs_pos.fetch_add(self.builder.step_angle().0, Relaxed);

                if self.pos() > self.limit_max().unwrap_or(PositionRad::INFINITY) {
                    self.builder.set_drive_mode(DriveMode::Stop, &mut self.ctrl)?;
                } 
            } else {
                self._state._abs_pos.fetch_sub(self.builder.step_angle().0, Relaxed);

                if self.pos() < self.limit_min().unwrap_or(PositionRad::NEG_INFINITY) {
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
        // Position 
            #[inline]
            fn pos(&self) -> PositionRad {
                self._state.pos()
            }   

            #[inline]
            fn overwrite_abs_pos(&mut self, pos : PositionRad) {
                self._state._abs_pos.store(pos.0, Relaxed);
            }
        //

        // U::Velocity
            #[inline]
            fn velocity_max(&self) -> Option<RadPerSecond> {
                self.builder.velocity_max()
            }

            #[inline]
            fn set_velocity_max(&mut self, velocity_opt : Option<RadPerSecond>) -> Result<(), ActuatorError> {
                self.builder.set_velocity_max(velocity_opt)?;
                Ok(())
            }
        //

        // Acceleration
            #[inline]
            fn acceleration_max(&self) -> Option<RadPerSecond2> {
                self.builder.acceleration_max()
            }

            fn set_acceleration_max(&mut self, acceleration_opt : Option<RadPerSecond2>) -> Result<(), ActuatorError> {
                self.builder.set_acceleration_max(acceleration_opt)?;
                Ok(())
            }
        //

        // Jolt
            fn jolt_max(&self) -> Option<RadPerSecond3> {
                self.builder.jolt_max()
            }

            fn set_jolt_max(&mut self, jolt_opt : Option<RadPerSecond3>) -> Result<(), ActuatorError> {
                self.builder.set_jolt_max(jolt_opt)?;
                Ok(())
            }
        //

        // Position limits
            #[inline]
            fn limit_max(&self) -> Option<PositionRad> {
                self._limit_max
            }

            #[inline]
            fn limit_min(&self) -> Option<PositionRad> {
                self._limit_min
            }
            
            #[inline]
            fn set_pos_limits(&mut self, min : Option<PositionRad>, max : Option<PositionRad>) {
                if let Some(min) = min {
                    self._limit_min = Some(min)
                }

                if let Some(max) = max {
                    self._limit_max = Some(max);
                }
            }

            #[inline]
            fn overwrite_pos_limits(&mut self, min : Option<PositionRad>, max : Option<PositionRad>) {
                self._limit_min = min;
                self._limit_max = max;
            }

            // TODO: Make a new output type, horrible idea to wrap all information into a single unit
            fn resolve_pos_limits_for_abs_pos(&self, pos : PositionRad) -> Radians {
                if let Some(ang) = self.limit_min() {
                    if pos < ang {
                        pos - ang
                    } else {
                        if let Some(ang) = self.limit_max() {
                            if pos > ang {
                                pos - ang
                            } else { 
                                Radians::ZERO 
                            }
                        } else {
                            Radians::ZERO
                        }
                    }
                } else {
                    if let Some(ang) = self.limit_max() {
                        if pos > ang {
                            pos - ang
                        } else { 
                            Radians::ZERO 
                        }
                    } else {
                        Radians::NAN
                    }
                }
            }

            fn set_endpos(&mut self, overwrite_abs_pos : PositionRad) {
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
        // State
            fn state(&self) -> &dyn SyncActuatorState {
                self._state.as_ref()
            }

            fn clone_state(&self) -> Arc<dyn SyncActuatorState> {
                self._state.clone()
            }
        // 

        fn drive_rel_blocking(&mut self, rel_dist : Radians, speed_f : Factor) -> Result<(), ActuatorError> {
            if !rel_dist.is_finite() {
                return Err(ActuatorError::InvaldRelativeDistance(rel_dist));
            }

            // Set drive mode, return mapped error if one occurs
            self.builder.set_drive_mode(DriveMode::FixedDistance(rel_dist, RadPerSecond::ZERO, speed_f), &mut self.ctrl)?;
            self.handle_builder()
        }

        fn drive_factor(&mut self, speed : Factor, direction : Direction) -> Result<(), ActuatorError> {
            // Set drive mode, return mapped error if one occurs
            self.builder.set_drive_mode(DriveMode::ConstFactor(speed, direction), &mut self.ctrl)?;
            self.handle_builder()
        }
    
        fn drive_speed(&mut self, speed : RadPerSecond) -> Result<(), ActuatorError> {
            // Set drive mode, return mapped error if one occurs
            self.builder.set_drive_mode(DriveMode::ConstVelocity(speed), &mut self.ctrl)?;
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

    impl<B : StepperBuilderAdvanced, C : StepperController> AdvancedActuator for StepperMotor<B, C> {
        // Loads
            fn force_gen(&self) -> NewtonMeters {
                self.builder.vars().force_load_gen
            }

            fn force_dir(&self) -> NewtonMeters {
                self.builder.vars().force_load_dir
            }

            fn apply_gen_force(&mut self, force : NewtonMeters) -> Result<(), ActuatorError> {
                self.builder.apply_gen_force(force)?;
                Ok(())
            }

            fn apply_dir_force(&mut self, force : NewtonMeters) -> Result<(), ActuatorError> {
                self.builder.apply_dir_force(force)?;
                Ok(())
            }

            fn inertia(&self) -> KgMeter2 {
                self.builder.vars().inertia_load
            }

            #[inline(always)]
            fn apply_inertia(&mut self, inertia : KgMeter2) -> Result<(), ActuatorError> {
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

    fn step_dist(&self) -> Radians {
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
    fn ptp_time_for_distance(&self, abs_pos_0 : PositionRad, abs_pos_t : PositionRad) -> Seconds {
        self.builder.ptp_time_for_distance(abs_pos_0, abs_pos_t)
    }
}