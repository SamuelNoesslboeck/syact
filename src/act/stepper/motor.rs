use core::ops::DerefMut;
use core::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::sync::mpsc::{Receiver, Sender, channel};

use atomic_float::AtomicF32;
use sylo::Direction;

use crate::{SyncActuator, Setup, lib_error};
use crate::act::{Interruptor, InterruptReason, Interruptible};
use crate::act::asyn::AsyncActuator;
use crate::act::stepper::{StepperActuator, StepperMotor, Controller, StepError};
use crate::data::{StepperConfig, StepperConst, ActuatorVars}; 
use crate::math::{HRCtrlStepBuilder, HRLimitedStepBuilder, HRStepBuilder};
use crate::math::force::torque_dyn;
use crate::units::*;


/// Message sent in the thread
pub enum AsyncMsg {
    //
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
type AsyncRes = Delta;

/// Error that is used if the async thread of the component has not been setup yet
#[inline(always)]
fn no_async() -> crate::Error {
    lib_error("Async has not been setup yet!")
}

/// A stepper motor
/// 
/// Controlled by two pins, one giving information about the direction, the other about the step signal (PWM)
pub struct HRStepper<C : Controller + Send + 'static> {
    /// Underlying controller of the stepper motor
    pub(crate) _ctrl : Arc<Mutex<C>>,

    /// All constants of the stepper motor
    _consts : StepperConst,
    /// Variables of the component
    _vars : ActuatorVars,

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
    _config : StepperConfig,

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

// Inits
impl<C : Controller + Send + 'static> HRStepper<C> {   
    /// Creates a new stepper controller with the given stepper motor constants `consts`
    pub fn new(device : C, consts : StepperConst) -> Self {
        Self { 
            _ctrl: Arc::new(Mutex::new(device)),
            _vars: ActuatorVars::ZERO, 
            
            _dir: Arc::new(AtomicBool::new(true)),
            _gamma : Arc::new(AtomicF32::new(0.0)),
            _step_ang: Arc::new(AtomicF32::new(consts.step_angle(1).0)),
            _micro: 1,

            _omega_max: Omega::ZERO,
            _t_step_cur: Arc::new(AtomicF32::new(Time::INFINITY.0)),

            _config: StepperConfig::ERROR,

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

    // Current omega
        pub fn omega_cur(&self) -> Omega {
            self._consts.omega(Time(self._t_step_cur.load(Ordering::Relaxed)), self.microsteps())
        }

        #[inline]
        pub fn set_omega_cur(&mut self, omega_cur : Omega) {
            self._t_step_cur.store(self.consts().step_time(omega_cur, self.microsteps()).0, Ordering::Relaxed)
        }
    // 

    // Debug
        pub fn debug_data(&self) {
            println!(" High-Resolution-Stepper ");
            println!("=========================");
            println!("self.vars = {:#?}", self.vars());
            println!("self.omega_max = {:#?}", self._omega_max);
            println!("self.micro = {:#?}", self._micro);
        }
    // 
}

// Basic functions
impl<C : Controller + Send + 'static> HRStepper<C> {
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
        I : Iterator<Item = Time>,
        F : FnMut() -> bool
    {
        // Record start gamma to return a delta distance at the end
        let gamma_0 = gamma.load(Ordering::Relaxed);      

        // Interruptors used for the movement process
        let mut intrs = intrs_mtx.lock().unwrap();

        Self::use_ctrl(ctrl_mtx, |device| -> Result<Delta, StepError> {
            // Drive each point in the curve
            for point in cur {
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
                device.step_no_wait(point)?;

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

    /// Setup all required parts for a fixed distance drive
    /// - Limit checks
    /// - sylo::Direction change
    fn setup_drive(&mut self, delta : Delta) -> Result<(), crate::Error> {
        let limit = self.limits_for_gamma(self.gamma() + delta);

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
            self._consts.step_angle(self._micro) 
        } else { 
            -self._consts.step_angle(self._micro) 
        };
        self.setup_drive(delta)?;

        Self::use_ctrl(&mut self._ctrl, |device| {
            device.step(time);
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

        Self::use_ctrl(&mut self._ctrl, |device| {
            device.set_dir(dir);
        }); 
    }
}

// Async helper functions
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
        let dir = self._dir.clone();
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
                    AsyncMsg::DriveFixed(mut cur) => {
                        // Set the active status
                        active.store(true, Ordering::Relaxed);

                        // Drive the curve with an interrupt function, that causes it to exit once a new message has been received
                        Self::drive_curve(&mut ctrl_mtx, &mut intrs_mtx,&gamma, &t_step_cur, &intr_reason,
                        Delta(step_ang.load(Ordering::Relaxed)), Direction::from_bool(dir.load(Ordering::Relaxed)), &mut cur, || {
                            // Check if a new message is available
                            if let Ok(msg_new) = receiver_thr.try_recv() {
                                // Set msg for next run
                                msg_prev = Some(msg_new);
                                true
                            } else {
                                false
                            }
                        }).unwrap();    // TODO: Handle

                        // Final step
                        Self::use_ctrl(&mut ctrl_mtx, |device| {
                            device.step_final().unwrap();     // TODO: Handle
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
                    AsyncMsg::DriveSpeed(mut cur, t_const) => {
                        // Set the active status
                        active.store(true, Ordering::Relaxed);

                        // Drive the curve with an interrupt function, that causes it to exit once a new message has been received
                        Self::drive_curve(&mut ctrl_mtx, &mut intrs_mtx,  &gamma, &t_step_cur, &intr_reason,
                        Delta(step_ang.load(Ordering::Relaxed)), Direction::from_bool(dir.load(Ordering::Relaxed)), &mut cur, || {
                            // Check if a new message is available
                            if let Ok(msg_new) = receiver_thr.try_recv() {
                                // Set msg for next run
                                msg_prev = Some(msg_new);
                                true
                            } else {
                                false
                            }
                        }).unwrap();    // TODO: Handle

                        if t_const.is_normal() {
                            // Update the current speed
                            t_step_cur.store(t_const.0, Ordering::Relaxed);

                            let mut intrs = intrs_mtx.lock().unwrap();
                            let mut interrupted = false;

                            loop {
                                // Check if a new message is available
                                if let Ok(msg_new) = receiver_thr.try_recv() {
                                    // Set msg for next run
                                    msg_prev = Some(msg_new);
                                    break;
                                }

                                // Run all interruptors
                                for intr in intrs.iter_mut() {
                                    if let Some(reason) = intr.check(&gamma) {
                                        intr_reason.lock().unwrap().replace(reason);
                                        interrupted = true;
                                        break;
                                    }
                                }

                                if interrupted {
                                    break;
                                }
    
                                Self::use_ctrl(&mut ctrl_mtx, |device| {
                                    device.step_no_wait(t_const).unwrap();    // TODO: Handle
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
        if self._config.voltage == 0.0 {
            return Err("Provide the component with vaild data! (`StepperConfig` is invalid)".into());
        }

        self._omega_max = self._consts.omega_max(self._config.voltage);

        self.setup_async();

        Ok(())
    }
}

impl<C : Controller + Send + 'static> SyncActuator for HRStepper<C> {
    // Data
        fn vars<'a>(&'a self) -> &'a ActuatorVars {
            &self._vars
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

            // Logging information
                log::debug!("[drive_rel (sync)] Delta: {:?}, Speed-Factor {:?}", delta, speed_f);
            // 
            
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
            cur.set_steps_max(self._consts.steps_from_angle_abs(delta, self._micro) - 1);

            // Set the active status to `true` 
            self.active.store(true, Ordering::Relaxed);

            let dir = self.dir();
    
            let delta = Self::drive_curve(
                &mut self._ctrl, 
                &mut self.intrs,         
                &self._gamma, 
                &self._t_step_cur, 
                &self._intr_reason,         
                Delta(self._step_ang.load(Ordering::Relaxed)), 
                dir, 
                &mut cur,  
                || { false }    
            ).map_err(|err| {
                // If the error is a step error, give debug information
                if !err.is_other() {
                    self.debug_data();
                    println!("{:#?}", &cur);
                }

                err
            })? + Delta(self._step_ang.load(Ordering::Relaxed));

            if self._intr_reason.lock().unwrap().is_none() {
                // Final (unpaused) step
                Self::use_ctrl(&mut self._ctrl, |device| -> Result<(), StepError> {
                    device.step_final()
                })?;

                // Update the pos after the final step
                self.set_gamma(self.gamma() + Delta(self._step_ang.load(Ordering::Relaxed)));
            }

            // Update the curret step time
            self.set_omega_cur(Omega::ZERO);

            // Unset active variable
            self.active.store(false, Ordering::Relaxed);
    
            Ok(delta)
        }

        fn drive_abs(&mut self, gamma : Gamma, speed_f : f32) -> Result<Delta, crate::Error> {
            // Logging
                log::debug!("[drive_abs (sync)] Gamma: {} (current: {}), Speed-Factor: {}", gamma, self.gamma(), speed_f);
            // 

            let delta = gamma - self.gamma();
            self.drive_rel(delta, speed_f)
        }
    // 

    // Async
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
            cur.set_steps_max(self._consts.steps_from_angle_abs(delta, self._micro) - 1);

            // Only execute if the async thread has been started yet
            self.drive_fixed_async(cur)?;
            self.set_active_status();

            Ok(())
        }

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
        fn set_gamma(&mut self, gamma : Gamma) {
            self._gamma.store(gamma.0, Ordering::Relaxed);
        }

        #[inline]
        fn omega_max(&self) -> Omega {
            self._omega_max
        }

        fn set_omega_max(&mut self, omega_max : Omega) {
            if omega_max > self._consts.omega_max(self._config.voltage) {
                panic!("Maximum omega must not be greater than recommended! (Given: {}, Rec: {})", omega_max, self._consts.omega_max(self._config.voltage));
            }

            self._omega_max = omega_max;
        }

        #[inline]
        fn set_limits(&mut self, min : Option<Gamma>, max : Option<Gamma>) {
            if min.is_some() {
                self._vars.lim.min = min;
            }

            if max.is_some() {
                self._vars.lim.max = max;
            }
        }

        #[inline]
        fn overwrite_limits(&mut self, min : Option<Gamma>, max : Option<Gamma>) {
            self._vars.lim.min = min;
            self._vars.lim.max = max;
        }

        fn limits_for_gamma(&self, gamma : Gamma) -> Delta {
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
            self.set_gamma(set_gamma);
    
            self.set_limits(
                if self.dir().as_bool() { self._vars.lim.min } else { Some(set_gamma) },
                if self.dir().as_bool() { Some(set_gamma) } else { self._vars.lim.max }
            )
        }
    //

    // Loads
        fn gen_force(&self) -> Force {
            self._vars.force_load_gen    
        }

        fn dir_force(&self) -> Force {
            self._vars.force_load_dir
        }

        #[inline(always)]
        fn apply_gen_force(&mut self, mut t : Force) -> Result<(), crate::Error> {
            t = t.abs();

            if t >= self._consts.torque_stall {
                return Err("Overload!".into());     // TODO: Improve msgs
            }

            self._vars.force_load_gen = t;

            Ok(())
        }

        #[inline(always)]
        fn apply_dir_force(&mut self, t : Force) -> Result<(), crate::Error> {
            if t.abs() >= self._consts.torque_stall { 
                return Err("Overload!".into());
            }

            self._vars.force_load_dir = t;

            Ok(())
        }

        fn inertia(&self) -> Inertia {
            self._vars.inertia_load
        }

        #[inline(always)]
        fn apply_inertia(&mut self, j : Inertia) {
            self._vars.inertia_load = j;
        }
    //
}

impl<C : Controller + Send + 'static> AsyncActuator for HRStepper<C> {
    type Duty = f32;
    
    fn drive(&mut self, dir : Direction, speed_f : f32) -> Result<(), crate::Error> {
        if (0.0 > speed_f) | (1.0 < speed_f) {
            panic!("Bad speed_f! {}", speed_f);
        }

        let omega_max = self.omega_max();
        let omega_0 = self.omega_cur();
        let omega_tar = omega_max;

        let mut builder = HRCtrlStepBuilder::from_builder(
            HRStepBuilder::from_motor(self, omega_0)
        );
        builder.set_speed_f(speed_f);

        // Constant time to hold
        let t_const = self._consts.step_time(omega_tar, self.microsteps()) / speed_f;

        if dir == self.dir() {
            builder.set_omega_tar(omega_tar)?;
            self.drive_speed_async(builder, t_const)?;
        } else {
            builder.set_omega_tar(Omega::ZERO)?;
            self.drive_speed_async(builder, Time::ZERO)?;

            self.await_inactive()?;         // TODO: Remove
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

    fn speed(&self) -> f32 {
        self.speed_f
    }
}

impl<C : Controller + Send + 'static> StepperActuator for HRStepper<C> {
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

        fn config(&self) -> &StepperConfig {
            &self._config
        }

        fn set_config(&mut self, config : StepperConfig) {
            self._config = config;
        }

        fn microsteps(&self) -> u8 {
            self._micro
        }

        fn set_microsteps(&mut self, micro : u8) {
            if micro == 0 {
                panic!("0 is not allowed as a microstep amount")
            }

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
            torque_dyn(self.consts(), omega, self._config.voltage, self.consts().current_max)
        }

        fn alpha_at_speed(&self, omega : Omega) -> Option<Alpha> {
            self._consts.alpha_max_for_omega(&self._vars, &self._config, omega, self.dir())
        }
    // 
}

impl<C : Controller + Send> Interruptible for HRStepper<C> {
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