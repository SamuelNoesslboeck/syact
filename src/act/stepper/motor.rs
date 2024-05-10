use core::marker::PhantomData;
use core::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{JoinHandle, self};
use std::sync::mpsc::{Receiver, Sender, channel};

use atomic_float::AtomicF32;
use syunit::*;

use crate::math::movements::DefinedActuator;
use crate::{SyncActuator, Setup, Dismantle};
use crate::act::{Interruptor, InterruptReason, Interruptible};
use crate::act::asyn::AsyncActuator;
use crate::act::stepper::{StepperActuator, Controller, StepperBuilder, DriveError, DriveMode};
use crate::data::{StepperConfig, StepperConst, MicroSteps}; 

pub enum AsyncMsg {
    Setup,
    Dismantle,
    Drive(DriveMode)
}

/// Distance moved by the thread
type AsyncRes = Result<(), DriveError>;

/// A stepper motor
/// 
/// Controlled by two pins, one giving information about the direction, the other about the step signal (PWM)
pub struct ThreadedStepper<B : StepperBuilder, C : Controller + Setup + Dismantle + Send + 'static> {
    builder : Arc<Mutex<B>>,

    // Atomics
        /// The current absolute position
        _gamma : Arc<AtomicF32>,
        active : Arc<AtomicBool>,
        _limit_min : Arc<AtomicF32>,
        _limit_max : Arc<AtomicF32>,
    // 

    /// Maxium velocity  of the component
    _microsteps : MicroSteps,

    // Threading and msg senders
    _thr : JoinHandle<()>,
    sender : Sender<AsyncMsg>,
    receiver : Receiver<AsyncRes>,

    // Interrupters
    intrs : Arc<Mutex<Vec<Box<dyn Interruptor + Send>>>>,
    _intr_reason : Arc<Mutex<Option<InterruptReason>>>,

    // Phantoms
    __pdc : PhantomData<C>
}

// Inits
impl<B : StepperBuilder + Send + 'static + 'static, C : Controller + Setup + Dismantle + Send + 'static> ThreadedStepper<B, C> {   
    /// Creates a new stepper controller with the given stepper motor constants `consts`
    pub fn new(mut ctrl : C, consts : StepperConst) -> Self {
        // Create communication channels
        let (sender_com, receiver_thr) : (Sender<AsyncMsg>, Receiver<AsyncMsg>) = channel();
        let (sender_thr, receiver_com) : (Sender<AsyncRes>, Receiver<AsyncRes>) = channel();

        // Shared
            let active = Arc::new(AtomicBool::new(false));
            let active_clone = active.clone();

            let builder = Arc::new(Mutex::new(B::new(consts)));
            let builder_clone = builder.clone();

            let gamma = Arc::new(AtomicF32::new(0.0));
            let gamma_clone = gamma.clone();

            let limit_min = Arc::new(AtomicF32::new(f32::NEG_INFINITY));
            let limit_min_clone = limit_min.clone();

            let limit_max = Arc::new(AtomicF32::new(f32::INFINITY));
            let limit_max_clone = limit_max.clone();

            let interruptors = Arc::new(Mutex::new(Vec::<Box<dyn Interruptor + Send + 'static>>::new()));
            let interruptors_clone = interruptors.clone();

            let interrupt_reason = Arc::new(Mutex::new(None));
            let interrupt_reason_clone = interrupt_reason.clone();
        // 

        let thr = thread::spawn(move || {
            // Variables
            let mut msg = AsyncMsg::Setup;
            let mut new_msg = false;

            loop {
                // If the builder is inactive, block until a new command is received
                if *builder.lock().unwrap().drive_mode() == DriveMode::Inactive {
                    if let Ok(m) = receiver_thr.recv() {
                        msg = m;
                    } 
                    new_msg = true;
                } else {
                    if let Ok(m) = receiver_thr.try_recv() {
                        msg = m;
                        new_msg = true;
                    }
                }

                if new_msg {
                    // Handle the incomming message, either with drive or setup
                    match &msg {
                        AsyncMsg::Setup => if let Err(_err) = ctrl.setup() {
                            // sender_thr.send(Err(err)).unwrap();
                            continue;
                        } else {
                            continue;
                        },
                        AsyncMsg::Dismantle => if let Err(_err) = ctrl.dismantle() {
                            // sender_thr.send(Err(err)).unwrap();
                            continue;
                        } else {
                            continue;
                        },
                        AsyncMsg::Drive(mode) => {
                            if let Err(err) = builder.lock().unwrap().set_drive_mode(mode.clone(), &mut ctrl) {
                                sender_thr.send(Err(err)).unwrap();
                            } else {
                                // Safety-Store
                                active.store(true, Ordering::Relaxed);
                            }
                        }
                    }   

                    new_msg = false;
                }

                let mut builder_ref = builder.lock().unwrap();

                if let Some(node) = builder_ref.next() {
                    let dir_val = builder_ref.dir();

                    let mut interrupted = false;

                    // Check all interruptors
                    for intr in interruptors.lock().unwrap().iter_mut() {
                        // Check if the direction is right
                        if let Some(i_dir) = intr.dir() {
                            if i_dir != dir_val {
                                continue;
                            }
                        }

                        if let Some(reason) = intr.check(&gamma) {
                            intr.set_temp_dir(Some(dir_val));
                            interrupt_reason.lock().unwrap().replace(reason);
                            sender_thr.send(Ok(())).unwrap();
                            
                            if let Err(err) = builder_ref.set_drive_mode(DriveMode::Stop, &mut ctrl) {
                                sender_thr.send(Err(err)).unwrap();
                            }
                            interrupted = true;
                        } else {
                            // Clear temp direction
                            intr.set_temp_dir(None);
                        }
                    }

                    if interrupted {
                        continue;   // Jump to top of loop
                    }

                    drop(builder_ref);

                    // Make step
                    if let Err(err) = ctrl.step_no_wait(node) {
                        sender_thr.send(Err(DriveError::Controller(err))).unwrap();
                    }

                    builder_ref = builder.lock().unwrap();

                    // Update gamma distance
                    if dir_val.as_bool() {
                        // TODO: Maybe add cache for step angle?
                        if gamma.fetch_add(builder_ref.step_angle().0, Ordering::Relaxed) > limit_max.load(Ordering::Relaxed) {
                            sender_thr.send(Err(DriveError::LimitReached)).unwrap();
                            if let Err(err) = builder_ref.set_drive_mode(DriveMode::Stop, &mut ctrl) {
                                sender_thr.send(Err(err)).unwrap();
                            }
                        } 
                    } else {
                        if gamma.fetch_sub(builder_ref.step_angle().0, Ordering::Relaxed) < limit_min.load(Ordering::Relaxed) {
                            sender_thr.send(Err(DriveError::LimitReached)).unwrap();
                            if let Err(err) = builder_ref.set_drive_mode(DriveMode::Stop, &mut ctrl) {
                                sender_thr.send(Err(err)).unwrap();
                            }
                        }
                    }

                    drop(builder_ref);
                } else {
                    if let Err(err) = builder_ref.set_drive_mode(DriveMode::Inactive, &mut ctrl) {
                        sender_thr.send(Err(err)).unwrap();
                    };

                    drop(builder_ref);

                    active.store(false, Ordering::Relaxed);
                    sender_thr.send(Ok(())).unwrap();
                }
            }
        });

        Self {
            builder: builder_clone,

            _gamma : gamma_clone,

            _limit_min: limit_min_clone,
            _limit_max: limit_max_clone,

            _microsteps: MicroSteps::default(),

            _thr: thr,
            sender: sender_com,
            receiver: receiver_com,

            active: active_clone,

            intrs : interruptors_clone,
            _intr_reason: interrupt_reason_clone,

            __pdc: PhantomData::default()
        }
    }

    /// Returns wheiter or not the stepper is actively moving
    #[inline]
    pub fn is_active(&self) -> bool {
        self.active.load(Ordering::Relaxed)
    }

    pub fn limit_min(&self) -> Option<Gamma> {
        let min = self._limit_min.load(Ordering::Relaxed);

        if min == f32::NEG_INFINITY {
            None
        } else {
            Some(Gamma(min))
        }
    }

    pub fn limit_max(&self) -> Option<Gamma> {
        let max = self._limit_max.load(Ordering::Relaxed);

        if max == f32::INFINITY {
            None
        } else {
            Some(Gamma(max))
        }
    }
}

impl<B : StepperBuilder + Send + 'static, C : Controller + Setup + Dismantle + Send + 'static> ThreadedStepper<B, C> {
    /// Returns the current direction of the motor
    pub fn dir(&self) -> Direction {
        self.builder.lock().unwrap().dir()
    }
}

impl<B : StepperBuilder + Send + 'static, C : Controller + Setup + Dismantle + Send + 'static> Setup for ThreadedStepper<B, C> {
    fn setup(&mut self) -> Result<(), crate::Error> {
        self.sender.send(AsyncMsg::Setup)?;
        Ok(())
    }
}

impl<B : StepperBuilder + Send + 'static, C : Controller + Setup + Dismantle + Send + 'static> Dismantle for ThreadedStepper<B, C> {
    fn dismantle(&mut self) -> Result<(), crate::Error> {
        self.sender.send(AsyncMsg::Dismantle)?;
        Ok(())
    }
}

impl<B : StepperBuilder + Send + 'static, C : Controller + Setup + Dismantle + Send + 'static> SyncActuator for ThreadedStepper<B, C> {
    // Movement
        fn drive_rel(&mut self, delta : Delta, speed : Factor) -> Result<(), crate::Error> {
            self.drive_rel_async(delta, speed)?;
            Ok(self.await_inactive()?)
        }

        fn drive_abs(&mut self, gamma : Gamma, speed : Factor) -> Result<(), crate::Error> {
            let delta = gamma - self.gamma();
            self.drive_rel(delta, speed)
        }
    // 

    // Async
        fn drive_rel_async(&mut self, delta : Delta, speed : Factor) -> Result<(), crate::Error> {
            // Check if the delta given is finite
            if !delta.is_finite() {
                return Err(format!("Invalid delta distance! {}", delta).into());
            }

            // Logging information
                log::debug!("[drive] Delta: {:?}, Speed-Factor {:?}", delta, speed);
            // 
            
            self.sender.send(AsyncMsg::Drive(DriveMode::FixedDistance(delta, Velocity::ZERO, speed)))?;
            self.active.store(true, Ordering::Relaxed);

            Ok(())
        }

        fn drive_abs_async(&mut self, gamma : Gamma, speed : Factor) -> Result<(), crate::Error> {
            let delta = gamma - self.gamma();
            self.drive_rel_async(delta, speed)
        }

        fn drive_velocity(&mut self, velocity_tar : Velocity) -> Result<(), crate::Error> {
            self.sender.send(AsyncMsg::Drive(DriveMode::ConstVelocity(velocity_tar.abs(), velocity_tar.get_direction())))?;
            Ok(())
        }
        
        fn await_inactive(&mut self) -> Result<(), crate::Error> {
            // TODO: Better message
            if !self.is_active() {
                return Ok(());      
            }

            self.receiver.recv()??;

            Ok(())
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
            self.builder.lock().unwrap().velocity_max()
        }

        fn set_velocity_max(&mut self, velocity_max : Velocity) {
            self.builder.lock().unwrap().set_velocity_max(velocity_max).unwrap();      // TODO
        }

        #[inline]
        fn set_limits(&mut self, min : Option<Gamma>, max : Option<Gamma>) {
            if let Some(min) = min {
                self._limit_min.store(min.into(), Ordering::Relaxed);
            }

            if let Some(max) = max {
                self._limit_max.store(max.into(), Ordering::Relaxed);
            }
        }

        #[inline]
        fn overwrite_limits(&mut self, min : Option<Gamma>, max : Option<Gamma>) {
            self._limit_min.store(min.unwrap_or(Gamma::NEG_INFINITY).into(), Ordering::Relaxed);
            self._limit_max.store(max.unwrap_or(Gamma::INFINITY).into(), Ordering::Relaxed);
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
            self.builder.lock().unwrap().vars().force_load_gen
        }

        fn force_dir(&self) -> Force {
            self.builder.lock().unwrap().vars().force_load_dir
        }

        fn apply_gen_force(&mut self, force : Force) -> Result<(), crate::Error> {
            self.builder.lock().unwrap().apply_gen_force(force)?;
            Ok(())
        }

        fn apply_dir_force(&mut self, force : Force) -> Result<(), crate::Error> {
            self.builder.lock().unwrap().apply_dir_force(force)?;
            Ok(())
        }

        fn inertia(&self) -> Inertia {
            self.builder.lock().unwrap().vars().inertia_load
        }

        #[inline(always)]
        fn apply_inertia(&mut self, inertia : Inertia) {
            self.builder.lock().unwrap().apply_inertia(inertia)
        }
    //
}

impl<B : StepperBuilder + Send + 'static, C : Controller + Setup + Dismantle + Send + 'static> AsyncActuator for ThreadedStepper<B, C> {
    fn drive(&mut self, _dir : Direction, _speed_f : Factor) -> Result<(), crate::Error> {
        todo!()
    }

    fn dir(&self) -> Direction {
        self.dir()
    }

    fn speed(&self) -> Factor {
        todo!()
    }
}

impl<B : StepperBuilder + Send + 'static, C : Controller + Setup + Dismantle + Send + 'static> StepperActuator for ThreadedStepper<B, C> 
where
    B : DefinedActuator 
{
    // Data
        fn consts(&self) -> &StepperConst {
            todo!()
        }

        fn config(&self) -> &StepperConfig {
            todo!()
        }

        fn set_config(&mut self, config : StepperConfig) {
            self.builder.lock().unwrap().set_config(config)
        }

        fn microsteps(&self) -> MicroSteps {
            self._microsteps
        }

        fn set_microsteps(&mut self, microsteps : MicroSteps) {
            self._microsteps = microsteps;
            
        }
    //

    fn step_ang(&self) -> Delta {
        self.builder.lock().unwrap().step_angle()
    }
}

impl<B : StepperBuilder + Send + 'static, C : Controller + Setup + Dismantle + Send + 'static> Interruptible for ThreadedStepper<B, C> {
    // Interruptors
        fn add_interruptor(&mut self, interruptor : Box<dyn Interruptor + Send>) {
            let mut intr = self.intrs.lock().unwrap();
            intr.push(interruptor);
        }

        fn intr_reason(&self) -> Option<InterruptReason> {
            // Return the value and replace it with `None`
            std::mem::replace(&mut self._intr_reason.lock().unwrap(), None)
        }
    // 
}

impl<B : StepperBuilder + Send + 'static, C : Controller + Setup + Dismantle + Send + 'static> DefinedActuator for ThreadedStepper<B, C> 
where
    B : DefinedActuator 
{
    fn ptp_time_for_distance(&self, gamma_0 : Gamma, gamma_t : Gamma) -> Time {
        self.builder.lock().unwrap().ptp_time_for_distance(gamma_0, gamma_t)
    }
}