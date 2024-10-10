use alloc::boxed::Box;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::sync::atomic::Ordering::Relaxed;

use syunit::*;

use crate::{AsyncActuator, Dismantle, Setup, SyncActuator};
use crate::act::{InterruptReason, Interruptible, Interruptor, SyncActuatorError, SyncActuatorState};
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
}

impl<B : StepperBuilder + Send + 'static, C : StepperController + Send + 'static> StepperMotor<B, C> {
    /// Returns the current direction of the motor
    pub fn dir(&self) -> Direction {
        self.builder.dir()
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
        // #################################
        // #    StepperMotor::drive_rel    #
        // #################################
        //
        // Main driving algorithm for stepper motors
        fn drive_rel(&mut self, delta : RelDist, speed_f : Factor) -> Result<(), SyncActuatorError> {
            if !delta.is_finite() {
                return Err(SyncActuatorError::InvaldDeltaDistance(delta));
            }

            // Set drive mode, return mapped error if one occurs
            if let Err(e) = self.builder.set_drive_mode(DriveMode::FixedDistance(delta, Velocity::ZERO, speed_f), &mut self.ctrl) {
                return Err(SyncActuatorError::StepperBuilderError(e));
            }

            while let Some(node) = self.builder.next() {
                let dir_val = self.builder.dir();

                // Check all interruptors
                for intr in self.interruptors.iter_mut() {
                    // Check if the direction is right
                    if let Some(i_dir) = intr.dir() {
                        if i_dir != dir_val {
                            continue;
                        }
                    }

                    if let Some(reason) = intr.check(self._state.gamma()) {
                        intr.set_temp_dir(Some(dir_val));
                        self._intr_reason.replace(reason);
                        
                        if let Err(e) = self.builder.set_drive_mode(DriveMode::Stop, &mut self.ctrl) {
                            return Err(SyncActuatorError::StepperBuilderError(e));
                        }
                    } else {
                        // Clear temp direction
                        intr.set_temp_dir(None);
                    }
                }

                // Make step and return error if occured
                if let Err(e) = self.ctrl.step(node) {
                    return Err(SyncActuatorError::StepperCtrlError(e));
                }

                // Check if the gamma value exeeds any limits
                if dir_val.as_bool() {
                    self._state._gamma.fetch_add(self.builder.step_angle().0, Relaxed);

                    if self.gamma() > self.limit_max().unwrap_or(AbsPos::INFINITY) {
                        if let Err(e) = self.builder.set_drive_mode(DriveMode::Stop, &mut self.ctrl) {
                            return Err(SyncActuatorError::StepperBuilderError(e))
                        }
                    } 
                } else {
                    self._state._gamma.fetch_sub(self.builder.step_angle().0, Relaxed);

                    if self.gamma() < self.limit_min().unwrap_or(AbsPos::NEG_INFINITY) {
                        if let Err(e) = self.builder.set_drive_mode(DriveMode::Stop, &mut self.ctrl) {
                            return Err(SyncActuatorError::StepperBuilderError(e))
                        }
                    } 
                }
            }

            Ok(())
        }

        /// Absolute movements derive from relative movements ([Self::drive_rel])
        fn drive_abs(&mut self, gamma : AbsPos, speed : Factor) -> Result<(), SyncActuatorError> {
            let delta = gamma - self.gamma();
            self.drive_rel(delta, speed)
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
        fn gamma(&self) -> AbsPos {
            self._state.gamma()
        }   

        #[inline]
        fn set_gamma(&mut self, gamma : AbsPos) {
            self._state._gamma.store(gamma.0, Relaxed);
        }

        #[inline]
        fn velocity_max(&self) -> Velocity {
            self.builder.velocity_max()
        }

        fn set_velocity_max(&mut self, velocity_max : Velocity) {
            self.builder.set_velocity_cap(velocity_max).unwrap();      // TODO
        }
    //

    // 
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

        fn resolve_pos_limits_for_gamma(&self, gamma : AbsPos) -> RelDist {
            if let Some(ang) = self.limit_min() {
                if gamma < ang {
                    gamma - ang
                } else {
                    if let Some(ang) = self.limit_max() {
                        if gamma > ang {
                            gamma - ang
                        } else { 
                            RelDist::ZERO 
                        }
                    } else {
                        RelDist::ZERO
                    }
                }
            } else {
                if let Some(ang) = self.limit_max() {
                    if gamma > ang {
                        gamma - ang
                    } else { 
                        RelDist::ZERO 
                    }
                } else {
                    RelDist::NAN
                }
            }
        }

        fn set_endpos(&mut self, set_gamma : AbsPos) {
            self.set_gamma(set_gamma);

            let dir = self.dir().as_bool();
    
            self.set_pos_limits(
                if dir { None } else { Some(set_gamma) },
                if dir { Some(set_gamma) } else { None }
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
    fn ptp_time_for_distance(&self, gamma_0 : AbsPos, gamma_t : AbsPos) -> Time {
        self.builder.ptp_time_for_distance(gamma_0, gamma_t)
    }
}

impl<B : StepperBuilder + Send + 'static, C : StepperController + Send + 'static> AsyncActuator for StepperMotor<B, C> 
{
    fn drive_factor(&mut self, dir : Direction, speed : Factor) -> Result<(), Self::Error> {
        
    }
}