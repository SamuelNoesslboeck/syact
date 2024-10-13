use alloc::boxed::Box;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::sync::atomic::Ordering::Relaxed;

use syunit::*;

use crate::{AsyncActuator, Dismantle, Setup, SyncActuator};
use crate::act::{InterruptReason, Interruptible, Interruptor, SyncActuatorError, SyncActuatorState};
use crate::act::asyn::{AsyncActuatorError, AsyncActuatorState};
use crate::act::stepper::{StepperActuator, StepperController, StepperBuilder, BuilderError, DriveMode, StepperState};
use crate::data::{StepperConfig, StepperConst, MicroSteps}; 
use crate::math::movements::DefinedActuator;

/// A stepper motor
/// 
/// Controlled by two pins, one giving information about the direction, the other about the step signal (PWM)
pub struct StepperMotor<B : StepperBuilder + Send + 'static, C : StepperController + Send + 'static> {
    builder : B,
    ctrl : C, 

    _state: Arc<StepperState>,

    _limit_min : Option<AbsPos>,
    _limit_max : Option<AbsPos>,

    // Interrupters
    interruptors : Vec<Box<dyn Interruptor + Send>>,
    _intr_reason : Option<InterruptReason>,
}

// Inits
impl<B : StepperBuilder + Send + 'static, C : StepperController + Send + 'static> StepperMotor<B, C> {   
    /// Creates a new stepper controller with the given stepper motor constants `consts`
    #[allow(unused_must_use)]
    pub fn new(ctrl : C, consts : StepperConst) -> Result<Self, BuilderError> {
        Ok(Self {
            builder: B::new(consts)?,
            ctrl,

            _state : Arc::new(StepperState::new()),

            _limit_min: None,
            _limit_max: None,

            interruptors : Vec::new(),
            _intr_reason: None
        })
    }

    /// ######################################
    /// #    StepperMotor::handle_builder    #
    /// ######################################
    ///
    /// Main driving algorithm for stepper motors, handles the builder until no nodes are left anymore
    /// 
    /// ## Thread
    /// 
    /// Blocks the current thread and creates the step signals until the builder is finished
    pub fn handle_builder(&mut self) -> Result<(), SyncActuatorError> {
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
                        
                        if let Err(e) = self.builder.set_drive_mode(DriveMode::Stop, &mut self.ctrl) {
                            return Err(SyncActuatorError::StepperBuilderError(e));
                        }
                    } else {
                        // Clear temporary direction
                        intr.set_temp_dir(None);
                    }
                }
            }

            // Make step and return error if occured
            if let Err(e) = self.ctrl.step(node) {
                return Err(SyncActuatorError::StepperCtrlError(e));
            }

            // Check if the abs_pos value exeeds any limits
            if direction.as_bool() {
                self._state._abs_pos.fetch_add(self.builder.step_angle().0, Relaxed);

                if self.abs_pos() > self.limit_max().unwrap_or(AbsPos::INFINITY) {
                    if let Err(e) = self.builder.set_drive_mode(DriveMode::Stop, &mut self.ctrl) {
                        return Err(SyncActuatorError::StepperBuilderError(e))
                    }
                } 
            } else {
                self._state._abs_pos.fetch_sub(self.builder.step_angle().0, Relaxed);

                if self.abs_pos() < self.limit_min().unwrap_or(AbsPos::NEG_INFINITY) {
                    if let Err(e) = self.builder.set_drive_mode(DriveMode::Stop, &mut self.ctrl) {
                        return Err(SyncActuatorError::StepperBuilderError(e))
                    }
                } 
            }
        }

        Ok(())
    }
}

impl<B : StepperBuilder + Send + 'static, C : StepperController + Send + 'static> StepperMotor<B, C> {
    /// Returns the current direction of the motor
    pub fn dir(&self) -> Direction {
        self.builder.direction()
    }
}

// Auto-Implement `Setup` and `Dismantle` of `StepperController`
    impl<B : StepperBuilder + Send + 'static, C : StepperController + Setup + Send + 'static> Setup for StepperMotor<B, C> {
        type Error = C::Error;

        fn setup(&mut self) -> Result<(), Self::Error> {
            self.ctrl.setup()?;
            Ok(())
        }
    }

    impl<B : StepperBuilder + Send + 'static, C : StepperController + Dismantle + Send + 'static> Dismantle for StepperMotor<B, C> {
        type Error = C::Error;

        fn dismantle(&mut self) -> Result<(), Self::Error> {
            self.ctrl.dismantle()?;
            Ok(())
        }
    }
//

impl<B : StepperBuilder + Send + 'static, C : StepperController + Send + 'static> SyncActuator for StepperMotor<B, C> {
    // Movement
        fn drive_rel(&mut self, rel_pos : RelDist, speed_f : Factor) -> Result<(), SyncActuatorError> {
            if !rel_pos.is_finite() {
                return Err(SyncActuatorError::InvaldDeltaDistance(rel_pos));
            }

            // Set drive mode, return mapped error if one occurs
            if let Err(e) = self.builder.set_drive_mode(DriveMode::FixedDistance(rel_pos, Velocity::ZERO, speed_f), &mut self.ctrl) {
                return Err(SyncActuatorError::StepperBuilderError(e));
            }

            self.handle_builder()
        }

        /// Absolute movements derive from relative movements ([Self::drive_rel])
        fn drive_abs(&mut self, abs_pos : AbsPos, speed : Factor) -> Result<(), SyncActuatorError> {
            let rel_pos = abs_pos - self.abs_pos();
            self.drive_rel(rel_pos, speed)
        }

        fn state(&self) -> &dyn SyncActuatorState {
            self._state.as_ref()
        }

        fn clone_state(&self) -> Arc<dyn SyncActuatorState> {
            self._state.clone()
        }
    // 

    // Position and velocity
        #[inline]
        fn abs_pos(&self) -> AbsPos {
            self._state.abs_pos()
        }   

        #[inline]
        fn set_abs_pos(&mut self, abs_pos : AbsPos) {
            self._state._abs_pos.store(abs_pos.0, Relaxed);
        }

        #[inline]
        fn velocity_max(&self) -> Velocity {
            self.builder.velocity_max()
        }

        fn set_velocity_max(&mut self, velocity_max : Velocity) {
            self.builder.set_velocity_cap(velocity_max).unwrap();      // TODO
        }
    //

    // Position limits
        fn limit_max(&self) -> Option<AbsPos> {
            self._limit_max
        }

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

        fn set_endpos(&mut self, set_abs_pos : AbsPos) {
            self.set_abs_pos(set_abs_pos);

            let dir = self.dir().as_bool();
    
            self.set_pos_limits(
                if dir { None } else { Some(set_abs_pos) },
                if dir { Some(set_abs_pos) } else { None }
            )
        }
    //

    // Loads
        fn force_gen(&self) -> Force {
            self.builder.vars().force_load_gen
        }

        fn force_dir(&self) -> Force {
            self.builder.vars().force_load_dir
        }

        fn apply_gen_force(&mut self, force : Force) -> Result<(), BuilderError> {
            self.builder.apply_gen_force(force)
        }

        fn apply_dir_force(&mut self, force : Force) -> Result<(), BuilderError> {
            self.builder.apply_dir_force(force)
        }

        fn inertia(&self) -> Inertia {
            self.builder.vars().inertia_load
        }

        #[inline(always)]
        fn apply_inertia(&mut self, inertia : Inertia) -> () {
            self.builder.apply_inertia(inertia).unwrap()
        }
    //
}

impl<B : StepperBuilder + Send + 'static, C : StepperController + Send + 'static> StepperActuator for StepperMotor<B, C> 
where
    B : DefinedActuator 
{
    // Data
        fn consts(&self) -> &StepperConst {
            self.builder.consts()
        }

        fn config(&self) -> &StepperConfig {
            self.builder.config()
        }

        fn set_config(&mut self, config : StepperConfig) -> Result<(), BuilderError> {
            self.builder.set_config(config)
        }

        fn microsteps(&self) -> MicroSteps {
            self.builder.microsteps()
        }

        fn set_microsteps(&mut self, microsteps : MicroSteps) -> Result<(), BuilderError> {
            self.builder.set_microsteps(microsteps)
            
        }
    //

    fn step_ang(&self) -> RelDist {
        self.builder.step_angle()
    }
}

impl<B : StepperBuilder + Send + 'static, C : StepperController + Send + 'static> Interruptible for StepperMotor<B, C> {
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

impl<B : StepperBuilder + Send + 'static, C : StepperController + Send + 'static> DefinedActuator for StepperMotor<B, C> 
where
    B : DefinedActuator 
{
    fn ptp_time_for_distance(&self, abs_pos_0 : AbsPos, abs_pos_t : AbsPos) -> Time {
        self.builder.ptp_time_for_distance(abs_pos_0, abs_pos_t)
    }
}

impl<B : StepperBuilder + Send + 'static, C : StepperController + Send + 'static> AsyncActuator for StepperMotor<B, C> 
{
    fn drive_factor(&mut self, dir : Direction, speed : Factor) -> Result<(), AsyncActuatorError> {
        // Set drive mode, return mapped error if one occurs
        if let Err(e) = self.builder.set_drive_mode(DriveMode::FixedDistance(rel_pos, Velocity::ZERO, speed_f), &mut self.ctrl) {
            return Err(SyncActuatorError::StepperBuilderError(e));
        }

    }

    fn drive_speed(&mut self, dir : Direction, speed : Velocity) -> Result<(), AsyncActuatorError> {
        
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