use core::ops::DerefMut;
use core::sync::atomic::{AtomicBool, Ordering};

// Include stuff for embedded threading if the feature is enabled
cfg_if::cfg_if! {
    if #[cfg(feature = "embed-thread")] {
        use std::sync::{Arc, Mutex};
        use std::thread::JoinHandle;
        use std::sync::mpsc::{Receiver, Sender, channel};

        use crate::comp::asyn::AsyncComp;
    } 
}

use atomic_float::AtomicF32;

use crate::{SyncComp, Setup, lib_error, Direction};
use crate::comp::stepper::{StepperComp, StepperMotor};
use crate::ctrl::{Controller, Interruptor, InterruptReason};
use crate::data::{CompData, StepperConst, CompVars}; 
use crate::math::{HRCtrlStepBuilder, HRLimitedStepBuilder, HRStepBuilder};
use crate::math::force::torque_dyn;
use crate::math::kin;
use crate::units::*;

/// Message sent in the thread
pub enum AsyncMsg {
    // /// 
    // DriveUnfixed(HRCtrlStepBuilder),
    /// Moves the stepper motor by a fixed distance 
    DriveFixed(HRLimitedStepBuilder),
    /// Moves the stepper motor at a constant speed
    DriveSpeed(HRCtrlStepBuilder, Time),
    /// Interrupts and stops the current movement
    Interrupt,
    /// Closes the async thread
    Close
}

/// Steps moved by the thread
#[cfg(feature = "embed-thread")]
type AsyncRes = Delta;

/// Error that is used if the async thread of the component has not been setup yet
#[cfg(feature = "embed-thread")]
#[inline(always)]
fn no_async() -> crate::Error {
    lib_error("Async has not been setup yet!")
}

// /// Error that is used when waiting for an inactive component
// #[inline(always)]
// #[cfg(feature = "std")]
// fn not_active() -> crate::Error {
//     lib_error("No movement has been started yet (Waiting for an inactive component!)")
// }

/// A stepper motor
/// 
/// Controlled by two pins, one giving information about the direction, the other about the step signal (PWM)
#[cfg(feature = "embed-thread")]
pub struct HRStepper<C : Controller + Send + 'static> {
    /// Underlying controller of the stepper motor
    pub(crate) _ctrl : Arc<Mutex<C>>,

    /// All constants of the stepper motor
    _consts : StepperConst,
    /// Variables of the component
    _vars : CompVars,

    /// The current direction of the stepper motor
    _dir : Arc<AtomicBool>,
    /// The current absolute position
    _gamma : Arc<AtomicF32>,
    /// The step angle of the motor
    _step_ang : Arc<AtomicF32>,
    /// Amount of microsteps in a full step
    _micro : u8,

    /// Maxium omega of the component
    _omega_max : Omega,
    _t_step_cur : Arc<AtomicF32>,

    /// Linked data shared between different components
    _data : CompData,

    // Threading and msg senders
    thr : Option<JoinHandle<()>>,
    sender : Option<Sender<AsyncMsg>>,
    receiver : Option<Receiver<AsyncRes>>,

    // Async driving
    active : Arc<AtomicBool>,
    speed_f : f32,

    // Interrupters
    intrs : Arc<Mutex<Vec<Box<dyn Interruptor + Send>>>>,
    _intr_reason : Arc<Mutex<Option<InterruptReason>>>
}

#[cfg(not(feature = "embed-thread"))]
pub struct HRStepper<C : Controller + Send> {
    /// Underlying controller of the stepper motor
    ctrl : C,

    /// All constants of the stepper motor
    consts : StepperConst,
    /// Variables of the component
    vars : CompVars,

    /// The current absolute position since set to a value
    pos : i64,
    micro : u8,

    omega_max : Omega,

    data : CompData
}

// Inits
impl<C : Controller + Send + 'static> HRStepper<C> {   
    /// Creates a new stepper controller with the given stepper motor constants `consts`
    pub fn new(ctrl : C, consts : StepperConst) -> Self {
        cfg_if::cfg_if! { if #[cfg(feature = "embed-thread")] {
            Self { 
                _ctrl: Arc::new(Mutex::new(ctrl)),
                _vars: CompVars::ZERO, 
                
                _dir: Arc::new(AtomicBool::new(true)),
                _gamma : Arc::new(AtomicF32::new(0.0)),
                _step_ang: Arc::new(AtomicF32::new(consts.step_ang(1).0)),
                _micro: 1,

                _omega_max: Omega::ZERO,
                _t_step_cur: Arc::new(AtomicF32::new(Omega::ZERO.0)),
    
                _data: CompData::ERROR,
    
                thr: None,
                sender: None,
                receiver: None,
    
                active: Arc::new(AtomicBool::new(false)),
                speed_f: 0.0,

                intrs : Arc::new(Mutex::new(Vec::new())),
                _intr_reason: Arc::new(Mutex::new(None)),

                // Would be moved too early if at the beginning of init
                _consts: consts 
            }
        } else {
            Self { 
                consts, 
                vars: CompVars::ZERO, 
    
                pos: 0,
                omega_max: Omega::ZERO,
    
                data: CompData::ERROR,
    
                sys: Pins {
                    dir: sys_dir,
                    step: sys_step
                }
            }
        }}
    }

    cfg_if::cfg_if! { if #[cfg(feature = "embed-thread")] {
        /// Function for accessing the ctrl substruct of a stepper motor with the embed-thread feature being enabled
        pub(crate) fn use_ctrl<F, R>(raw_ctrl : &mut Arc<Mutex<C>>, func : F) -> R 
        where F: FnOnce(&mut C) -> R {
            let mut ctrl_ref = raw_ctrl.lock().unwrap();
            let ctrl = ctrl_ref.deref_mut();

            func(ctrl)
        }
    } else {
        /// Function for accessing the ctrl substruct of a stepper motor with the embed-thread feature being enabled
        pub(crate) fn use_ctrl<F, R>(raw_ctrl : C, func : F) -> R 
        where F: FnOnce(&mut C) -> R {
            func(raw_ctrl)
        }
    } }

    /// Returns wheiter or not the stepper is actively moving
    #[inline]
    pub fn is_active(&self) -> bool {
        self.active.load(Ordering::Relaxed)
    }

    // Current omega
        pub fn omega_cur(&self) -> Omega {
            self._consts.omega(Time(self._t_step_cur.load(Ordering::Relaxed)), self.micro())
        }

        #[inline]
        pub fn set_omega_cur(&mut self, omega_cur : Omega) {
            self._t_step_cur.store(self.consts().step_time(omega_cur, self.micro()).0, Ordering::Relaxed)
        }
    // 
}

// Basic functions
impl<C : Controller + Send + 'static> HRStepper<C> {
    /// Write a curve of signals to the step output pins
    /// The curve can be any iterator that yields `Time`
    pub fn drive_curve<I : Iterator<Item = Time>, F : FnMut() -> bool>(ctrl_mtx : &mut Arc<Mutex<C>>, intrs_mtx : &mut Arc<Mutex<Vec<Box<dyn Interruptor + Send>>>>, 
        gamma : &Arc<AtomicF32>, t_step_cur : &Arc<AtomicF32>, intr_reason : &Arc<Mutex<Option<InterruptReason>>>, step_ang : Delta, cur : I, mut h_func : F) -> Delta
    {
        // Record start gamma to return a delta distance at the end
        let gamma_0 = gamma.load(Ordering::Relaxed);      

        // Interruptors used for the movement process
        let mut intrs = intrs_mtx.lock().unwrap();

        Self::use_ctrl(ctrl_mtx, |ctrl| {
            // Drive each point in the curve
            for point in cur {
                // Run all interruptors
                for intr in intrs.iter_mut() {
                    if let Some(reason) = intr.check(gamma) {
                        intr_reason.lock().unwrap().replace(reason);
                        return Delta(gamma.load(Ordering::Relaxed) - gamma_0); 
                    }
                }

                // Additional helper function
                if h_func() {
                    return Delta(gamma.load(Ordering::Relaxed) - gamma_0); 
                }

                // Run step
                ctrl.step_no_wait(point);

                // Update the current speed
                t_step_cur.store(point.0, Ordering::Relaxed);
                
                // Update the position after each step
                gamma.store(
                    gamma.load(Ordering::Relaxed) + step_ang.0, 
                    Ordering::Relaxed
                );
            }

            Delta(gamma.load(Ordering::Relaxed) - gamma_0)
        })
    }

    /// Setup all required parts for a fixed distance drive
    /// - Limit checks
    /// - Direction change
    fn setup_drive(&mut self, delta : Delta) -> Result<(), crate::Error> {
        let limit = self.lim_for_gamma(self.gamma() + delta);

        // Checks if the given limit is reached 
        if limit.is_normal() {
            return Err(lib_error(format!("The given delta reaches outside the limit! (by {})", limit)))
        }
        
        // Changes direction based on distance sign
        //  => Stays inactive if delta == 0
        if delta > Delta::ZERO {
            self.set_dir(Direction::CW);
        } else if delta < Delta::ZERO {
            self.set_dir(Direction::CCW);
        }

        // Reset the interrupt reason
        self._intr_reason.lock().unwrap().take();

        Ok(())
    }
}

impl<C : Controller + Send + 'static> HRStepper<C> {
    /// Makes the component move a single step with the given `time`
    /// 
    /// # Error
    /// 
    /// Returns an error if `setup_drive()` fails
    pub fn step(&mut self, time : Time) -> Result<(), crate::Error> {
        let delta = if self.dir().as_bool() { 
            self._consts.step_ang(self._micro) 
        } else { 
            -self._consts.step_ang(self._micro) 
        };
        self.setup_drive(delta)?;

        Self::use_ctrl(&mut self._ctrl, |ctrl| {
            ctrl.step(time);
        });

        Ok(())
    }

    /// Returns the current direction of the motor
    pub fn dir(&self) -> Direction {
        Direction::from_bool(self._dir.load(Ordering::Relaxed))
    }

    /// Sets the driving direction of the component. Note that the direction of the motor depends on the connection of the cables.
    /// The "dir"-pin is directly set to the value given
    #[inline(always)]
    pub fn set_dir(&mut self, dir : Direction) {
        self._dir.store(dir.as_bool(), Ordering::Relaxed);
        
        // Update the step angle to match directions
        if dir.as_bool() {
            self._step_ang.store(self.step_ang().0.abs(), Ordering::Relaxed);
        } else {
            self._step_ang.store(-self.step_ang().0.abs(), Ordering::Relaxed);
        }

        Self::use_ctrl(&mut self._ctrl, |ctrl| {
            ctrl.set_dir(dir);
        }); 
    }
}

// Async helper functions
#[cfg(feature = "embed-thread")]
impl<C : Controller + Send + 'static> HRStepper<C> {
    /// Sets the active status of the component
    fn set_active_status(&mut self) {
        self.active.store(true, Ordering::Relaxed)
    }

    /// Cleans the "active"-status of the stepper motor, enabling new movements
    fn reset_active_status(&mut self) -> Result<(), crate::Error> {
        let recv : &Receiver<AsyncRes>;

        // Checks wheiter async has been setup yet
        if let Some(_recv) = &self.receiver {
            recv = _recv;
        } else {
            return Err(no_async());
        }

        loop {
            // Receive all messages
            if recv.try_recv().is_err() {
                break;
            }
        }

        // Reset the active variable
        self.active.store(false, Ordering::Relaxed);

        Ok(())
    }

    /// Starts the async-embedded-thread used for driving multiple movements at a time
    /// - A thread is more practical here, as the driving process itself is pretty simple and async should be used for "many complex processes"
    fn setup_async(&mut self) {
        // Create communication channels
        let (sender_com, receiver_thr) : (Sender<AsyncMsg>, Receiver<AsyncMsg>) = channel();
        let (sender_thr, receiver_com) : (Sender<AsyncRes>, Receiver<AsyncRes>) = channel();

        // Clone the data required for the process
        let mut ctrl_mtx = self._ctrl.clone();
        let mut intrs_mtx = self.intrs.clone();

        let gamma = self._gamma.clone();
        let t_step_cur = self._t_step_cur.clone();
        let active = self.active.clone();
        let step_ang = self._step_ang.clone();
        let intr_reason = self._intr_reason.clone();

        // Initialize the thread
        self.thr = Some(std::thread::spawn(move || {
            // Message of a previous iteration
            // - Used for example when a new message is received and the old thread stops
            let mut msg_prev : Option<_> = None;
            let mut msg : AsyncMsg;
        
            loop {
                // Check if a message has been stored up
                //  => Force to receive new one otherwise
                if let Some(msg_r) = msg_prev { 
                    msg = msg_r;
                    msg_prev = None;
                } else {
                    msg = match receiver_thr.recv() {
                        Ok(msg_new) => msg_new,
                        Err(_) => {
                            // TODO: Error handling
                            // println!("Error occured in thread! {}", err.to_string());   // TODO: Improve error message
                            break;
                        }
                    };
                }

                let pos_0 = gamma.load(Ordering::Relaxed);

                match msg {
                    AsyncMsg::DriveFixed(cur) => {
                        // Set the active status
                        active.store(true, Ordering::Relaxed);

                        // Drive the curve with an interrupt function, that causes it to exit once a new message has been received
                        Self::drive_curve(&mut ctrl_mtx, &mut intrs_mtx,&gamma, &t_step_cur, &intr_reason,
                        Delta(step_ang.load(Ordering::Relaxed)), cur, || {
                            // Check if a new message is available
                            if let Ok(msg_new) = receiver_thr.try_recv() {
                                // Set msg for next run
                                msg_prev = Some(msg_new);
                                true
                            } else {
                                false
                            }
                        });

                        // Final step
                        Self::use_ctrl(&mut ctrl_mtx, |ctrl| {
                            ctrl.step_final();
                        });

                        // Update the position after the final step
                        gamma.store(
                            gamma.load(Ordering::Relaxed) + step_ang.load(Ordering::Relaxed), 
                            Ordering::Relaxed
                        );

                        // Update the current speed after final step
                        t_step_cur.store(Time::INFINITY.0, Ordering::Relaxed);

                        // Unset the active status
                        active.store(false, Ordering::Relaxed);

                        // Send a message back to the main thread to report the distance travelled
                        sender_thr.send(Delta(gamma.load(Ordering::Relaxed) - pos_0)).unwrap();  // TODO: Handle result
                    },
                    AsyncMsg::DriveSpeed(cur, t_const) => {
                        // Set the active status
                        active.store(true, Ordering::Relaxed);

                        // Drive the curve with an interrupt function, that causes it to exit once a new message has been received
                        Self::drive_curve(&mut ctrl_mtx, &mut intrs_mtx,  &gamma, &t_step_cur, &intr_reason,
                        Delta(step_ang.load(Ordering::Relaxed)), cur, || {
                            // Check if a new message is available
                            if let Ok(msg_new) = receiver_thr.try_recv() {
                                // Set msg for next run
                                msg_prev = Some(msg_new);
                                true
                            } else {
                                false
                            }
                        });

                        if t_const.is_normal() {
                            // Update the current speed
                            t_step_cur.store(t_const.0, Ordering::Relaxed);

                            loop {
                                // Check if a new message is available
                                if let Ok(msg_new) = receiver_thr.try_recv() {
                                    // Set msg for next run
                                    msg_prev = Some(msg_new);
                                    break;
                                }
    
                                Self::use_ctrl(&mut ctrl_mtx, |ctrl| {
                                    ctrl.step_no_wait(t_const);
                                }); 

                                // Update position
                                gamma.store(
                                    gamma.load(Ordering::Relaxed) + step_ang.load(Ordering::Relaxed), 
                                    Ordering::Relaxed
                                );
                            };
                        }

                        // Unset the active status
                        active.store(false, Ordering::Relaxed);

                        // Send a message back to the main thread to report the distance travelled
                        sender_thr.send(Delta(gamma.load(Ordering::Relaxed) - pos_0)).unwrap();   // TODO: Handle result
                    },
                    AsyncMsg::Interrupt => {
                        // Do nothing as it is supposed to interrupt the
                    },
                    AsyncMsg::Close => {
                        // Break out of the main loop to end the thread
                        break;
                    }
                };
            };
        }));

        self.sender = Some(sender_com);
        self.receiver = Some(receiver_com);
    }   

    // Communication
        /// Drive a fixed curve described with a `HRLimitedStepBuilder` in the embedded thread
        fn drive_fixed_async(&mut self, cur : HRLimitedStepBuilder) -> Result<(), crate::Error> {
            // Only execute if the sender has been initialized
            if let Some(sender) = &self.sender {
                sender.send(AsyncMsg::DriveFixed(cur))?;
                Ok(())
            } else {
                Err(no_async())
            }
        }

        /// Drive a fixed speed 
        fn drive_speed_async(&mut self, cur : HRCtrlStepBuilder, t_const : Time) -> Result<(), crate::Error> {
            // Only execute if the sender has been initialized
            if let Some(sender) = &self.sender {
                sender.send(AsyncMsg::DriveSpeed(cur, t_const))?;
                Ok(())
            } else {
                Err(no_async())
            }
        }

        /// Interrupt the current movement process being executed by the async embedded thread
        pub fn interrupt_async(&mut self) -> Result<(), crate::Error> {
            // Only execute if the sender has been initialized
            if let Some(sender) = &self.sender {
                sender.send(AsyncMsg::Interrupt)?;
                Ok(())
            } else {
                Err(no_async())
            }
        }

        /// Close the async embedded thread
        pub fn close_async(&mut self) -> Result<(), crate::Error> {
            // Only execute if the sender has been initialized
            if let Some(sender) = &self.sender {
                sender.send(AsyncMsg::Close)?;
                Ok(())
            } else {
                Err(no_async())
            }
        }
    // 
}

impl<C : Controller + Send + 'static> Setup for HRStepper<C> {
    fn setup(&mut self) -> Result<(), crate::Error> {
        if self._data.u == 0.0 {
            return Err("Provide the component with vaild data! (`CompData` is invalid)".into());
        }

        self._omega_max = self._consts.omega_max(self._data.u);

        #[cfg(feature = "std")]
        self.setup_async();

        Ok(())
    }
}

impl<C : Controller + Send + 'static> SyncComp for HRStepper<C> {
    // Data
        fn vars<'a>(&'a self) -> &'a CompVars {
            &self._vars
        }

        fn data<'a>(&'a self) -> &'a CompData {
            &self._data
        }

        #[inline]
        fn write_data(&mut self, data : CompData) {
            self._data = data;
        }  
    // 

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

    // Movement
        fn drive_rel(&mut self, delta : Delta, speed_f : f32) -> Result<Delta, crate::Error> {
            // Check for invalid speed factor (out of bounds)
            if (1.0 < speed_f) | (0.0 >= speed_f) {
                panic!("Invalid speed factor! {}", speed_f)
            }
            
            // Check if the delta given is finite
            if !delta.is_finite() {
                return Err(format!("Invalid delta distance! {}", delta).into());
            }

            // If delta is zero, do nothing
            if delta == Delta::ZERO {
                return Ok(Delta::ZERO)   
            }
            
            self.setup_drive(delta)?;
    
            let mut cur = HRLimitedStepBuilder::from_builder(
                HRStepBuilder::from_motor(self, Omega::ZERO)
            );
    
            cur.set_omega_tar(self.omega_max())?;
            cur.set_speed_f(speed_f);

            // The builder generates a curve with one step less then the distance required, 
            // because for each step signal there has to be a waittime afterwars in order to move correctly.
            // The last step is made manually, as it is the halting step to stop the curve
            //
            // Curve with 6 steps total
            // Step     Waittime            Last manual step
            // \/         \/                     \/
            // | ----- | --- | -- | --- | --\-- |
            cur.set_steps_max(self._consts.steps_from_ang_abs(delta, self._micro) - 1);

            // Set the active status to `true` 
            self.active.store(true, Ordering::Relaxed);
    
            let delta = Self::drive_curve(
                &mut self._ctrl, &mut self.intrs,           // Mutexes
                &self._gamma, &self._t_step_cur, &self._intr_reason,         // Atomics
                Delta(self._step_ang.load(Ordering::Relaxed)), cur,  // The curve and step angle
                || { false }        // (Not required) helper function
            ) + Delta(self._step_ang.load(Ordering::Relaxed));
            
            // Final (unpaused) step
            Self::use_ctrl(&mut self._ctrl, |ctrl| {
                ctrl.step_final();
            });

            // Update the pos after the final step
            self.write_gamma(self.gamma() + Delta(self._step_ang.load(Ordering::Relaxed)));

            // Update the curret step time
            self.set_omega_cur(Omega::ZERO);

            // Unset active variable
            self.active.store(false, Ordering::Relaxed);
    
            Ok(delta)
        }

        fn drive_abs(&mut self, gamma : Gamma, speed_f : f32) -> Result<Delta, crate::Error> {
            let delta = gamma - self.gamma();
            self.drive_rel(delta, speed_f)
        }
    // 

    // Async
        #[cfg(feature = "std")]
        fn drive_rel_async(&mut self, delta : Delta, speed_f : f32) -> Result<(), crate::Error> {
            // Check for invalid speed factor (out of bounds)
            if (1.0 < speed_f) | (0.0 >= speed_f) {
                panic!("Invalid speed factor! {}", speed_f)
            }
            
            // Check if the delta given is finite
            if !delta.is_finite() {
                return Err(format!("Invalid delta distance! {}", delta).into());
            }

            self.reset_active_status()?;

            // If delta is zero, do nothing
            if delta == Delta::ZERO {
                return Ok(())    
            }
            
            self.setup_drive(delta)?;
    
            let mut cur = HRLimitedStepBuilder::from_builder(
                HRStepBuilder::from_motor(self, Omega::ZERO)
            );
    
            cur.set_omega_tar(self.omega_max())?;
            cur.set_speed_f(speed_f);

            // The builder generates a curve with one step less then the distance required, 
            // because for each step signal there has to be a waittime afterwars in order to move correctly.
            // The last step is made manually, as it is the halting step to stop the curve
            //
            // Curve with 6 steps total
            // Step     Waittime            Last manual step
            // \/         \/                     \/
            // | ----- | --- | -- | --- | --\-- |
            cur.set_steps_max(self._consts.steps_from_ang_abs(delta, self._micro) - 1);

            // Only execute if the async thread has been started yet
            self.drive_fixed_async(cur)?;
            self.set_active_status();

            Ok(())
        }

        #[cfg(feature = "std")]
        fn drive_abs_async(&mut self, gamma : Gamma, speed_f : f32) -> Result<(), crate::Error> {
            let delta = gamma - self.gamma();
            self.drive_rel_async(delta, speed_f)
        }

        fn drive_omega(&mut self, omega_tar : Omega) -> Result<(), crate::Error> {
            self.drive(if omega_tar.is_sign_negative() { 
                Direction::CCW
            } else {
                Direction::CW
            }, (omega_tar / self.omega_max()).abs())
        }
        
        #[cfg(feature = "std")]
        fn await_inactive(&mut self) -> Result<Delta, crate::Error> {
            if self.receiver.is_none() {
                return Err(no_async());
            }

            // TODO: Make sure nothing is blocking
            // if !self.is_active() {
            //     return Err(not_active());
            // }

            let delta = if let Some(recv) = &self.receiver {
                recv.recv().unwrap()    // TODO: Remove unwrap
            } else {
                Delta::NAN
            };

            self.active.store(false, Ordering::Relaxed);

            Ok(delta)
        }
    //

    // Position
        #[inline]
        fn gamma(&self) -> Gamma {
            Gamma(self._gamma.load(Ordering::Relaxed))
        }   

        #[inline]
        fn write_gamma(&mut self, gamma : Gamma) {
            #[cfg(feature = "std")] {
                self._gamma.store(gamma.0, Ordering::Relaxed);
            }

            #[cfg(not(feature = "std"))] {
                self.pos = self.consts.steps_from_ang(gamma - Gamma::ZERO);
            }
        }

        #[inline]
        fn omega_max(&self) -> Omega {
            self._omega_max
        }

        fn set_omega_max(&mut self, omega_max : Omega) {
            if omega_max > self._consts.omega_max(self._data.u) {
                #[cfg(feature = "std")]
                panic!("Maximum omega must not be greater than recommended! (Given: {}, Rec: {})", omega_max, self._consts.omega_max(self._data.u));
            }

            self._omega_max = omega_max;
        }

        #[inline]
        fn set_limit(&mut self, min : Option<Gamma>, max : Option<Gamma>) {
            if min.is_some() {
                self._vars.lim.min = min;
            }

            if max.is_some() {
                self._vars.lim.max = max;
            }
        }

        #[inline]
        fn reset_limit(&mut self, min : Option<Gamma>, max : Option<Gamma>) {
            self._vars.lim.min = min;
            self._vars.lim.max = max;
        }

        fn lim_for_gamma(&self, gamma : Gamma) -> Delta {
            match self._vars.lim.min {
                Some(ang) => {
                    if gamma < ang {
                        gamma - ang
                    } else {
                        match self._vars.lim.max {
                            Some(ang) => {
                                if gamma > ang {
                                    gamma - ang
                                } else { Delta::ZERO }
                            },
                            None => Delta::ZERO
                        }
                    }
                },
                None => match self._vars.lim.max {
                    Some(ang) => {
                        if gamma > ang {
                            gamma - ang
                        } else { Delta::ZERO }
                    },
                    None => Delta::NAN
                }
            }
        }

        fn set_end(&mut self, set_gamma : Gamma) {
            self.write_gamma(set_gamma);
    
            self.set_limit(
                if self.dir().as_bool() { self._vars.lim.min } else { Some(set_gamma) },
                if self.dir().as_bool() { Some(set_gamma) } else { self._vars.lim.max }
            )
        }
    //

    // Loads
        #[inline(always)]
        fn apply_inertia(&mut self, j : Inertia) {
            self._vars.j_load = j;
        }

        #[inline(always)]
        fn apply_force(&mut self, t : Force) {      // TODO: Overloads
            if t >= self._consts.t_s {
                // TODO: Notify
                // println!("Load will not be applied! {}", t);     
                return;
            }

            self._vars.t_load = t;
        }

        #[inline(always)]
        fn apply_bend_f(&mut self, f_bend : f32) {
            self._vars.bend_f = f_bend;
        }
    //
}

#[cfg(feature = "embed-thread")]
impl<C : Controller + Send + 'static> AsyncComp for HRStepper<C> {
    fn drive(&mut self, dir : Direction, speed_f : f32) -> Result<(), crate::Error> {
        if (0.0 > speed_f) | (1.0 < speed_f) {
            panic!("Bad speed_f! {}", speed_f);
        }

        let omega_max = self.omega_max();
        let omega_0 = self.omega_cur();
        let omega_tar = omega_max;

        // println!(" => Building curve: o_max: {}, o_0: {}, o_tar: {}, spf_0: {}, spf: {}", 
        //     self.omega_max, omega_0, omega_tar, self.speed_f, speed_f);

        let mut builder = HRCtrlStepBuilder::from_builder(
            HRStepBuilder::from_motor(self, omega_0)
        );
        builder.set_speed_f(speed_f);

        // Constant time to hold
        let t_const = self._consts.step_time(omega_tar, self.micro()) / speed_f;

        if dir == self.dir() {
            builder.set_omega_tar(omega_tar)?;
            self.drive_speed_async(builder, t_const)?;
        } else {
            builder.set_omega_tar(Omega::ZERO)?;
            self.drive_speed_async(builder, Time::ZERO)?;

            self.await_inactive()?;
            self.set_dir(dir);

            builder = HRCtrlStepBuilder::from_builder(
                HRStepBuilder::from_motor(self, omega_0)
            );    

            builder.set_omega_tar(omega_tar)?;
            builder.set_speed_f(speed_f);

            self.drive_speed_async(builder, t_const)?;
        }

        self.speed_f = speed_f;

        Ok(())
    }

    fn dir(&self) -> Direction {
        self.dir()
    }

    fn speed_f(&self) -> f32 {
        self.speed_f
    }
}

impl<C : Controller + Send + 'static> StepperComp for HRStepper<C> {
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
            &self._consts
        }

        fn micro(&self) -> u8 {
            self._micro
        }

        fn set_micro(&mut self, micro : u8) {
            self._step_ang.store(self.step_ang().0 * (self._micro as f32) / (micro as f32), Ordering::Relaxed);
            self._micro = micro;
        }
    //

    fn step_ang(&self) -> Delta {
        Delta(self._step_ang.load(Ordering::Relaxed))
    }
}

impl<C : Controller + Send + 'static> StepperMotor for HRStepper<C> {
    // Calculations
        fn torque_at_speed(&self, omega : Omega) -> Force {
            torque_dyn(self.consts(), omega, self._data.u)
        }

        fn alpha_at_speed(&self, omega : Omega) -> Result<Alpha, crate::Error> {
            self.consts().alpha_max_dyn(self.torque_at_speed(omega), self.vars())
        }

        fn approx_time_ptp(&self, delta : Delta, speed_f : f32, acc : usize) -> Result<Time, crate::Error> {
            let omega_max = self.omega_max() * speed_f;
    
            let alpha_av = self.alpha_av(Omega::ZERO, omega_max, acc)?;
    
            let time_min = delta / omega_max;       // * 2.0 for average, / 2.0 for one side => * 1.0
            let time_accel = omega_max / alpha_av;
            
            if time_accel > time_min {
                // Maximum omega is not reached
                Ok(kin::accel_from_zero(delta / 2.0, alpha_av) * 2.0)
            } else {
                // Maximum omega is reached
                let accel_dist = time_accel * omega_max / 2.0;
                Ok(time_accel * 2.0 + (delta - accel_dist * 2.0) / omega_max)
            }
        }

        fn alpha_av(&self, omega_0 : Omega, omega_tar : Omega, acc : usize) -> Result<Alpha, crate::Error> {
            let mut alpha_sum = self.alpha_at_speed(omega_0)? + self.alpha_at_speed(omega_tar)?;
            let omega_diff = (omega_tar - omega_0) / (acc as f32);
    
            for i in 1 .. acc {
                alpha_sum += self.alpha_at_speed(omega_0 + omega_diff * (i as f32))?;
            }
    
            Ok(alpha_sum / (2.0 + acc as f32))
        }
    // 

    fn drive_nodes(&mut self, delta : Delta, omega_0 : Omega, _omega_tar : Omega, _corr : &mut (Delta, Time)) -> Result<(), crate::Error> {
        self.setup_drive(delta)?;

        let mut _builder = HRStepBuilder::from_motor(self, omega_0);
        // let curve = builder.to_speed_lim(delta, omega_0, omega_tar, corr)?;

        // dbg!(self.consts.steps_from_ang(delta));
        // dbg!(2.0 * delta / (omega_0 + omega_tar));
        // dbg!(curve.len());a
        // dbg!(curve.iter().map(|x| x.0).sum::<f32>());

        drop(_builder);

        // self.drive_curve_async(curve, false, None)?;

        Ok(())
    }
}