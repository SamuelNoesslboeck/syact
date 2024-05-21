use core::sync::atomic::Ordering;
use std::sync::Arc;

use atomic_float::AtomicF32;
use syunit::*;

use crate::{SyncActuator, Setup, Dismantle};
use crate::act::{InterruptReason, Interruptible, Interruptor, SyncActuatorError, SyncDriveFuture};
use crate::act::stepper::{StepperActuator, StepperController, StepperBuilder, BuilderError, DriveMode};
use crate::data::{StepperConfig, StepperConst, MicroSteps}; 
use crate::math::movements::DefinedActuator;

/// A stepper motor
/// 
/// Controlled by two pins, one giving information about the direction, the other about the step signal (PWM)
pub struct StepperMotor<B : StepperBuilder, C : StepperController + Setup + Dismantle + Send + 'static> {
    builder : B,
    ctrl : C, 

    // Atomics
        /// The current absolute position
        _gamma : Arc<AtomicF32>,
    // 

    _limit_min : Option<Gamma>,
    _limit_max : Option<Gamma>,

    // Interrupters
    interruptors : Vec<Box<dyn Interruptor + Send>>,
    _intr_reason : Option<InterruptReason>,
}

// Inits
impl<B : StepperBuilder + Send + 'static + 'static, C : StepperController + Setup + Dismantle + Send + 'static> StepperMotor<B, C> {   
    /// Creates a new stepper controller with the given stepper motor constants `consts`
    #[allow(unused_must_use)]
    pub fn new(ctrl : C, consts : StepperConst) -> Result<Self, BuilderError> {
        Ok(Self {
            builder: B::new(consts)?,
            ctrl,

            _gamma : Arc::new(AtomicF32::new(0.0)),

            _limit_min: None,
            _limit_max: None,

            interruptors : vec![],
            _intr_reason: None
        })
    }

    // /// Returns wheiter or not the stepper is actively moving
    // #[inline]
    // pub fn is_active(&self) -> bool {
    //     self.active.load(Ordering::Relaxed)
    // }

    pub fn limit_min(&self) -> Option<Gamma> {
        self._limit_min
    }

    pub fn limit_max(&self) -> Option<Gamma> {
        self._limit_max
    }
}

impl<B : StepperBuilder + Send + 'static, C : StepperController + Setup + Dismantle + Send + 'static> StepperMotor<B, C> {
    /// Returns the current direction of the motor
    pub fn dir(&self) -> Direction {
        self.builder.dir()
    }
}

impl<B : StepperBuilder + Send + 'static, C : StepperController + Setup + Dismantle + Send + 'static> Setup for StepperMotor<B, C> {
    fn setup(&mut self) -> Result<(), crate::Error> {
        self.ctrl.setup()?;
        Ok(())
    }
}

impl<B : StepperBuilder + Send + 'static, C : StepperController + Setup + Dismantle + Send + 'static> Dismantle for StepperMotor<B, C> {
    fn dismantle(&mut self) -> Result<(), crate::Error> {
        self.ctrl.dismantle()?;
        Ok(())
    }
}

impl<B : StepperBuilder + Send + 'static, C : StepperController + Setup + Dismantle + Send + 'static> SyncActuator for StepperMotor<B, C> {
    // Movement
        fn drive_rel(&mut self, delta : Delta, speed_f : Factor) -> SyncDriveFuture {
            if !delta.is_finite() {
                return SyncDriveFuture::Done(Err(SyncActuatorError::InvaldDeltaDistance(delta)));
            }

            // Set drive mode, return mapped error if one occurs
            if let Err(e) = self.builder.set_drive_mode(DriveMode::FixedDistance(delta, Velocity::ZERO, speed_f), &mut self.ctrl) {
                return SyncDriveFuture::Done(Err(SyncActuatorError::StepperBuilderError(e)));
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

                    if let Some(reason) = intr.check(&self._gamma) {
                        intr.set_temp_dir(Some(dir_val));
                        self._intr_reason.replace(reason);
                        
                        if let Err(e) = self.builder.set_drive_mode(DriveMode::Stop, &mut self.ctrl) {
                            return SyncDriveFuture::Done(Err(SyncActuatorError::StepperBuilderError(e)));
                        }
                    } else {
                        // Clear temp direction
                        intr.set_temp_dir(None);
                    }
                }

                // Make step and return error if occured
                if let Err(e) = self.ctrl.step_no_wait(node) {
                    return SyncDriveFuture::Done(Err(SyncActuatorError::StepperCtrlError(e)));
                }
                
                // Update gamma value
                self._gamma.fetch_add(self.builder.step_angle().0, Ordering::Relaxed);

                // Check if the gamma value exeeds any limits
                if dir_val.as_bool() {
                    if self.gamma() > self.limit_max().unwrap_or(Gamma::INFINITY) {
                        if let Err(e) = self.builder.set_drive_mode(DriveMode::Stop, &mut self.ctrl) {
                            return SyncDriveFuture::Done(Err(SyncActuatorError::StepperBuilderError(e)))
                        }
                    } 
                } else {
                    if self.gamma() < self.limit_min().unwrap_or(Gamma::NEG_INFINITY) {
                        if let Err(e) = self.builder.set_drive_mode(DriveMode::Stop, &mut self.ctrl) {
                            return SyncDriveFuture::Done(Err(SyncActuatorError::StepperBuilderError(e)))
                        }
                    } 
                }
            }

            SyncDriveFuture::Done(Ok(()))
        }

        fn drive_abs(&mut self, gamma : Gamma, speed : Factor) -> SyncDriveFuture {
            let delta = gamma - self.gamma();
            self.drive_rel(delta, speed)
        }
    // 

    // Position
        #[inline]
        fn gamma(&self) -> Gamma {
            Gamma(self._gamma.load(Ordering::Relaxed))
        }   

        #[inline]
        fn set_gamma(&mut self, gamma : Gamma) {
            self._gamma.store(gamma.0, Ordering::Relaxed);
        }

        #[inline]
        fn velocity_max(&self) -> Velocity {
            self.builder.velocity_max()
        }

        fn set_velocity_max(&mut self, velocity_max : Velocity) {
            self.builder.set_velocity_cap(velocity_max).unwrap();      // TODO
        }

        #[inline]
        fn set_limits(&mut self, min : Option<Gamma>, max : Option<Gamma>) {
            if let Some(min) = min {
                self._limit_min = Some(min)
            }

            if let Some(max) = max {
                self._limit_max = Some(max);
            }
        }

        #[inline]
        fn overwrite_limits(&mut self, min : Option<Gamma>, max : Option<Gamma>) {
            self._limit_min = min;
            self._limit_max = max;
        }

        fn limits_for_gamma(&self, gamma : Gamma) -> Delta {
            if let Some(ang) = self.limit_min() {
                if gamma < ang {
                    gamma - ang
                } else {
                    if let Some(ang) = self.limit_max() {
                        if gamma > ang {
                            gamma - ang
                        } else { 
                            Delta::ZERO 
                        }
                    } else {
                        Delta::ZERO
                    }
                }
            } else {
                if let Some(ang) = self.limit_max() {
                    if gamma > ang {
                        gamma - ang
                    } else { 
                        Delta::ZERO 
                    }
                } else {
                    Delta::NAN
                }
            }
        }

        fn set_end(&mut self, set_gamma : Gamma) {
            self.set_gamma(set_gamma);

            let dir = self.dir().as_bool();
    
            self.set_limits(
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

        fn apply_gen_force(&mut self, force : Force) -> Result<(), crate::Error> {
            self.builder.apply_gen_force(force)?;
            Ok(())
        }

        fn apply_dir_force(&mut self, force : Force) -> Result<(), crate::Error> {
            self.builder.apply_dir_force(force)?;
            Ok(())
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

impl<B : StepperBuilder + Send + 'static, C : StepperController + Setup + Dismantle + Send + 'static> StepperActuator for StepperMotor<B, C> 
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

    fn step_ang(&self) -> Delta {
        self.builder.step_angle()
    }
}

impl<B : StepperBuilder + Send + 'static, C : StepperController + Setup + Dismantle + Send + 'static> Interruptible for StepperMotor<B, C> {
    // Interruptors
        fn add_interruptor(&mut self, interruptor : Box<dyn Interruptor + Send>) {
            self.interruptors.push(interruptor);
        }

        fn intr_reason(&mut self) -> Option<InterruptReason> {
            // Return the value and replace it with `None`
            std::mem::replace(&mut self._intr_reason, None)
        }
    // 
}

impl<B : StepperBuilder + Send + 'static, C : StepperController + Setup + Dismantle + Send + 'static> DefinedActuator for StepperMotor<B, C> 
where
    B : DefinedActuator 
{
    fn ptp_time_for_distance(&self, gamma_0 : Gamma, gamma_t : Gamma) -> Time {
        self.builder.ptp_time_for_distance(gamma_0, gamma_t)
    }
}