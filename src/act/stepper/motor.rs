use core::marker::PhantomData;
use core::ops::DerefMut;
use core::ptr::NonNull;
use core::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{JoinHandle, self};
use std::sync::mpsc::{Receiver, Sender, channel};

use atomic_float::AtomicF32;
use sylo::Direction;

use crate::{SyncActuator, Setup, Dismantle};
use crate::act::{Interruptor, InterruptReason, Interruptible};
use crate::act::asyn::AsyncActuator;
use crate::act::stepper::{StepperActuator, StepperMotor, Controller, StepError, StepperBuilder, DriveError, DriveMode};
use crate::data::{StepperConfig, StepperConst, ActuatorVars, SpeedFactor, MicroSteps}; 
use crate::units::*;

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

    /// The current direction of the stepper motor
    _dir : Arc<AtomicBool>,
    /// The current absolute position
    _gamma : Arc<AtomicF32>,

    /// Maxium omega of the component
    _t_step_cur : Arc<AtomicF32>,
    _microsteps : MicroSteps,

    // Threading and msg senders
    thr : JoinHandle<()>,
    sender : Sender<AsyncMsg>,
    receiver : Receiver<AsyncRes>,

    // Async driving
    active : Arc<AtomicBool>,

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

        let dir = Arc::new(AtomicBool::new(true));
        let dir_clone = dir.clone();

        let gamma = Arc::new(AtomicF32::new(0.0));
        let gamma_clone = gamma.clone();

        let thr = thread::spawn(move || {
            // Variables
            let mut msg = AsyncMsg::Setup;
            let mut new_msg = false;


            loop {
                // If the builder is inactive, block until a new command is received
                if *builder.lock().unwrap().drive_mode() == DriveMode::Inactive {
                    msg = receiver_thr.recv().unwrap();     // TODO: Remove unwrap
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
                        },
                        AsyncMsg::Dismantle => if let Err(_err) = ctrl.dismantle() {
                            // sender_thr.send(Err(err)).unwrap();
                            continue;
                        },
                        AsyncMsg::Drive(mode) => {
                            if let Err(err) = builder.lock().unwrap().set_drive_mode(mode.clone(), &mut ctrl) {
                                sender_thr.send(Err(err)).unwrap();
                            } else {
                                active.store(true, Ordering::Relaxed);
                            }
                        }
                    }   

                    new_msg = false;
                }

                if let Some(node) = builder.lock().unwrap().next() {
                    if let Err(err) = ctrl.step_no_wait(node) {
                        sender_thr.send(Err(DriveError::Step(err))).unwrap();
                    }

                    if dir.load(Ordering::Relaxed) {
                        // TODO: Maybe add cache for step angle?
                        gamma.fetch_add(builder.lock().unwrap().step_angle().0, Ordering::Relaxed);
                    } else {
                        gamma.fetch_sub(builder.lock().unwrap().step_angle().0, Ordering::Relaxed);
                    }
                } else {
                    if let Err(err) = builder.lock().unwrap().set_drive_mode(DriveMode::Inactive, &mut ctrl) {
                        sender_thr.send(Err(err)).unwrap();
                    };
                    active.store(false, Ordering::Relaxed);
                    sender_thr.send(Ok(())).unwrap();
                }
            }
        });

        Self {
            builder: builder_clone,

            _dir: dir_clone,
            _gamma : gamma_clone,

            _t_step_cur: Arc::new(AtomicF32::new(Time::INFINITY.0)),
            _microsteps: MicroSteps::default(),

            thr,
            sender: sender_com,
            receiver: receiver_com,

            active: active_clone,

            intrs : Arc::new(Mutex::new(Vec::new())),
            _intr_reason: Arc::new(Mutex::new(None)),

            __pdc: PhantomData::default()
        }
    }

    /// Function for accessing the device substruct of a stepper motor with the embed-thread feature being enabled
    pub(crate) fn use_ctrl<F, R>(raw_ctrl : &mut Arc<Mutex<C>>, func : F) -> R 
    where F: FnOnce(&mut C) -> R {
        let mut ctrl_ref = raw_ctrl.lock().unwrap();
        let device = ctrl_ref.deref_mut();

        func(device)
    }

    /// Returns wheiter or not the stepper is actively moving
    #[inline]
    pub fn is_active(&self) -> bool {
        self.active.load(Ordering::Relaxed)
    }
}

// Basic functions
impl<B : StepperBuilder + Send + 'static, C : Controller + Setup + Dismantle + Send + 'static> ThreadedStepper<B, C> {
    /// Write a curve of signals to the step output pins
    /// The curve can be any iterator that yields `Time`
    pub fn drive_curve<I, F>(
        ctrl_mtx : &mut Arc<Mutex<C>>,                                      // Mutex to the controller
        intrs_mtx : &mut Arc<Mutex<Vec<Box<dyn Interruptor + Send>>>>,      // Mutex to the vector of interruptors
        gamma : &Arc<AtomicF32>,                                            // Atomic gamma value
        t_step_cur : &Arc<AtomicF32>,                                       // Atomic for current time
        intr_reason : &Arc<Mutex<Option<InterruptReason>>>,                 // Atomic for interrupt reason
        step_ang : Delta,                                                   // Current step angle used
        dir : Direction,                                                    // Current movement direction
        cur : &mut I,                                                            // The curve
        mut h_func : F                                                      // Helper function for additional functinality
    ) -> Result<Delta, StepError>    
    where
        I : Iterator<Item = Time> + core::fmt::Debug + Clone,
        F : FnMut() -> bool
    {
        // Record start gamma to return a delta distance at the end
        let gamma_0 = gamma.load(Ordering::Relaxed);      

        // Interruptors used for the movement process
        let mut intrs = intrs_mtx.lock().unwrap();

        Self::use_ctrl(ctrl_mtx, |device| -> Result<Delta, StepError> {
            // Drive each point in the curve
            for (i, point) in cur.enumerate() {
                // Run all interruptors
                for intr in intrs.iter_mut() {
                    // Check if the direction is right
                    if let Some(i_dir) = intr.dir() {
                        if i_dir != dir {
                            continue;
                        }
                    }

                    if let Some(reason) = intr.check(gamma) {
                        intr.set_temp_dir(Some(dir));
                        intr_reason.lock().unwrap().replace(reason);
                        return Ok(Delta(gamma.load(Ordering::Relaxed) - gamma_0)); 
                    } else {
                        // Clear temp direction
                        intr.set_temp_dir(None);
                    }
                }

                // Additional helper function
                if h_func() {
                    return Ok(Delta(gamma.load(Ordering::Relaxed) - gamma_0)); 
                }

                // Run step
                device.step_no_wait(point).map_err(|err| {
                    // Additional debug information
                    log::error!("Steperror occured! {}", err);
                    log::error!("Dir: {:?}, Step-Number: {:?}", dir, i);
                    err
                })?;

                // Update the current speed
                t_step_cur.store(point.0, Ordering::Relaxed);
                
                // Update the position after each step
                gamma.store(
                    gamma.load(Ordering::Relaxed) + step_ang.0, 
                    Ordering::Relaxed
                );
            }

            Ok(Delta(gamma.load(Ordering::Relaxed) - gamma_0))
        })
    }
}

impl<B : StepperBuilder + Send + 'static, C : Controller + Setup + Dismantle + Send + 'static> ThreadedStepper<B, C> {
    /// Returns the current direction of the motor
    pub fn dir(&self) -> Direction {
        Direction::from_bool(self._dir.load(Ordering::Relaxed))
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
    // Data
        fn vars<'a>(&'a self) -> &'a ActuatorVars {
            todo!()
        }
    // 

    // Movement
        fn drive_rel(&mut self, delta : Delta, speed : SpeedFactor) -> Result<(), crate::Error> {
            self.drive_rel_async(delta, speed)?;
            Ok(self.await_inactive()?)
        }

        fn drive_abs(&mut self, gamma : Gamma, speed : SpeedFactor) -> Result<(), crate::Error> {
            let delta = gamma - self.gamma();
            self.drive_rel(delta, speed)
        }
    // 

    // Async
        fn drive_rel_async(&mut self, delta : Delta, speed : SpeedFactor) -> Result<(), crate::Error> {
            // Check if the delta given is finite
            if !delta.is_finite() {
                return Err(format!("Invalid delta distance! {}", delta).into());
            }

            // If delta or speed_f is zero, do nothing
            if self.consts().steps_from_angle(delta, self.microsteps()) == 0 {
                return Ok(())   
            }

            // Logging information
                log::debug!("[drive] Delta: {:?}, Speed-Factor {:?}", delta, speed);
            // 
            
            self.sender.send(AsyncMsg::Drive(DriveMode::FixedDistance(delta, Omega::ZERO, speed)))?;

            Ok(())
        }

        fn drive_abs_async(&mut self, gamma : Gamma, speed : SpeedFactor) -> Result<(), crate::Error> {
            let delta = gamma - self.gamma();
            self.drive_rel_async(delta, speed)
        }

        fn drive_omega(&mut self, omega_tar : Omega) -> Result<(), crate::Error> {
            self.sender.send(AsyncMsg::Drive(DriveMode::ConstOmega(omega_tar.abs(), omega_tar.get_direction())))?;
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
        fn omega_max(&self) -> Omega {
            self.builder.lock().unwrap().omega_max()
        }

        fn set_omega_max(&mut self, omega_max : Omega) {
            self.builder.lock().unwrap().set_omega_max(omega_max).unwrap();      // TODO
        }

        #[inline]
        fn set_limits(&mut self, min : Option<Gamma>, max : Option<Gamma>) {
            todo!()
        }

        #[inline]
        fn overwrite_limits(&mut self, min : Option<Gamma>, max : Option<Gamma>) {
            todo!()
        }

        fn limits_for_gamma(&self, gamma : Gamma) -> Delta {
            todo!()
            // match self._vars.lim.min {
            //     Some(ang) => {
            //         if gamma < ang {
            //             gamma - ang
            //         } else {
            //             match self._vars.lim.max {
            //                 Some(ang) => {
            //                     if gamma > ang {
            //                         gamma - ang
            //                     } else { Delta::ZERO }
            //                 },
            //                 None => Delta::ZERO
            //             }
            //         }
            //     },
            //     None => match self._vars.lim.max {
            //         Some(ang) => {
            //             if gamma > ang {
            //                 gamma - ang
            //             } else { Delta::ZERO }
            //         },
            //         None => Delta::NAN
            //     }
            // }
        }

        fn set_end(&mut self, set_gamma : Gamma) {
            todo!()
            // self.set_gamma(set_gamma);
    
            // self.set_limits(
            //     if self.dir().as_bool() { self._vars.lim.min } else { Some(set_gamma) },
            //     if self.dir().as_bool() { Some(set_gamma) } else { self._vars.lim.max }
            // )
        }
    //

    // Loads
        fn gen_force(&self) -> Force {
            todo!()
            // self._vars.force_load_gen    
        }

        fn dir_force(&self) -> Force {
            todo!()
        }

        #[inline(always)]
        fn apply_gen_force(&mut self, mut t : Force) -> Result<(), crate::Error> {
            todo!()
        }

        #[inline(always)]
        fn apply_dir_force(&mut self, t : Force) -> Result<(), crate::Error> {
            todo!()
        }

        fn inertia(&self) -> Inertia {
            todo!()
        }

        #[inline(always)]
        fn apply_inertia(&mut self, j : Inertia) {
            todo!()
        }
    //
}

impl<B : StepperBuilder + Send + 'static, C : Controller + Setup + Dismantle + Send + 'static> AsyncActuator for ThreadedStepper<B, C> {
    type Duty = SpeedFactor;
    
    fn drive(&mut self, dir : Direction, speed_f : SpeedFactor) -> Result<(), crate::Error> {
        todo!()
    }

    fn dir(&self) -> Direction {
        self.dir()
    }

    fn speed(&self) -> SpeedFactor {
        todo!()
    }
}

impl<B : StepperBuilder + Send + 'static, C : Controller + Setup + Dismantle + Send + 'static> StepperActuator for ThreadedStepper<B, C> {
    // Motor
        fn motor(&self) -> &dyn StepperMotor {
            self
        }

        fn motor_mut(&mut self) -> &mut dyn StepperMotor {
            self
        }
    // 

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

impl<B : StepperBuilder + Send + 'static, C : Controller + Setup + Dismantle + Send + 'static> StepperMotor for ThreadedStepper<B, C> {
    // Calculations
        fn torque_at_speed(&self, omega : Omega) -> Force {
            todo!()
        }

        fn alpha_at_speed(&self, omega : Omega) -> Option<Alpha> {
            todo!()
        }
    // 
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