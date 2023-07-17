use core::ops::{AddAssign, DerefMut, MulAssign, Deref};

// Include stuff for embedded threading if the feature is enabled
cfg_if::cfg_if! {
    if #[cfg(feature = "embed-thread")] {
        use std::sync::{Arc, Mutex};
        use std::thread::JoinHandle;
        use std::sync::mpsc::{Receiver, Sender, channel};

        use crate::comp::asyn::AsyncComp;
    } 
}

use crate::{SyncComp, Setup, lib_error, Direction};
use crate::comp::stepper::StepperComp;
use crate::ctrl::{Controller, Interrupter};
use crate::data::{LinkedData, StepperConst, CompVars}; 
use crate::math::{StepTimeBuilder, CurveBuilder, CtrlStepTimeBuilder};
use crate::meas::MeasData;
use crate::units::*;

/// Elements are:
/// - the curve to drive : `Vec<Time>`
/// - interuptable : `bool`
/// - constant time to drive after curve : `Option<Time>`
#[cfg(feature = "std")]
type AsyncMsg = (Vec<Time>, bool, Option<Time>);
/// Steps moved by the thread
#[cfg(feature = "std")]
type AsyncRes = i64;

/// Error that is used if the async thread of the component has not been setup yet
#[cfg(feature = "std")]
#[inline(always)]
fn no_async() -> crate::Error {
    lib_error("Async has not been setup yet!")
}

/// Error that is used when waiting for an inactive component
#[inline(always)]
#[cfg(feature = "std")]
fn not_active() -> crate::Error {
    lib_error("No movement has been started yet (Waiting for an inactive component!)")
}

/// A stepper motor
/// 
/// Controlled by two pins, one giving information about the direction, the other about the step signal (PWM)
#[derive(Debug)]
#[cfg(feature = "embed-thread")]
pub struct HRStepper<C : Controller + Send + Send> {
    /// Underlying controller of the stepper motor
    ctrl : Arc<Mutex<C>>,

    /// All constants of the stepper motor
    consts : StepperConst,
    /// Variables of the component
    vars : CompVars,

    /// The current absolute position since set to a value
    pos : Arc<Mutex<i64>>,
    /// Amount of microsteps in a full step
    micro : u8,

    /// Maxium omega of the component
    omega_max : Omega,

    /// Linked data shared between different components
    lk : LinkedData,
    
    thr : Option<JoinHandle<()>>,
    sender : Option<Sender<Option<AsyncMsg>>>,
    receiver : Option<Receiver<AsyncRes>>,

    active : bool,
    speed_f : f32
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

    lk : LinkedData
}

// Inits
impl<C : Controller + Send> HRStepper<C> {   
    /// Creates a new stepper controller with the given stepper motor constants `consts`
    pub fn new(ctrl : C, consts : StepperConst) -> Self {
        cfg_if::cfg_if!(if #[cfg(feature = "embed-thread")] {
            Self { 
                ctrl: Arc::new(Mutex::new(ctrl)),
                consts, 
                vars: CompVars::ZERO, 
    
                pos : Arc::new(Mutex::new(0)),
                micro: 1,
                omega_max: Omega::ZERO,
    
                lk: LinkedData { u: 0.0, s_f: 0.0 },
    
                thr: None,
                sender: None,
                receiver: None,
    
                active: false,
                speed_f: 0.0
            }
        } else {
            Self { 
                consts, 
                vars: CompVars::ZERO, 
    
                pos: 0,
                omega_max: Omega::ZERO,
    
                lk: LinkedData { u: 0.0, s_f: 0.0 },
    
                sys: Pins {
                    dir: sys_dir,
                    step: sys_step
                }
            }
        })
    }

    /// Creates a new structure with both pins set to [pin::ERR_PIN] just for simulation and testing purposes
    #[inline]
    pub fn new_sim(ctrl : C, data : StepperConst) -> Self {
        Self::new(ctrl, data)
    }

    pub fn use_ctrl<F, R>(&mut self, mut func : F) -> R
    where F: FnMut(&mut C) -> R {
        cfg_if::cfg_if!(if #[cfg(feature = "embed-thread")] {
            let ctrl = self.ctrl.lock().unwrap().deref_mut();
        } else {
            let ctrl = &mut ctrl;
        });

        func(ctrl)
    }

    pub fn pos(&self) -> i64 {
        cfg_if::cfg_if!(if #[cfg(feature = "embed-thread")] {
            return self.pos.lock().unwrap().clone();
        } else {
            return self.pos;
        })
    }

    pub fn dir(&self) -> Direction {
        cfg_if::cfg_if!(if #[cfg(feature = "embed-thread")] {
            return self.ctrl.lock().unwrap().dir();
        } else {
            return self.ctrl.dir();
        })
    }

    pub fn set_pos(&mut self, pos : i64) {
        cfg_if::cfg_if!(if #[cfg(feature = "embed-thread")] {
            *self.pos.lock().unwrap().deref_mut() = pos;
        } else {
            self.pos = pos;
        })
    }
}

// Basic functions
impl<C : Controller + Send> HRStepper<C> {
    /// Write a curve of signals to the output pin
    pub fn drive_curve<I : Iterator<Item = Time>>(&mut self, cur : I) -> i64 {
        self.use_ctrl(|ctrl| {
            let mut pos_0 = self.pos();
    
            for point in cur {
                ctrl.step(point);
                self.set_pos(self.pos() + if self.dir().as_bool() { 1 } else { -1 });
            }

            self.pos() - pos_0
        })
    }

    /// Drive the given curve with a possibility to interrupt the movement by e.g. a measurement
    fn drive_curve_int<I : Iterator<Item = Time>>(&mut self, mut cur : I, intr : Interrupter, intr_data : &mut dyn MeasData) -> (usize, bool) {
        self.use_ctrl(|ctrl| {
            let mut trav = 0;

            for point in cur {
                if intr(intr_data) {
                    break;
                }

                ctrl.step(point);
                self.set_pos(self.pos() + if self.dir().as_bool() { 1 } else { -1 });
                trav += 1;
            }
    
            ( trav, cur.next().is_some() )  
        })
    }

    fn setup_drive(&mut self, delta : Delta) -> Result<(), crate::Error> {
        let limit = self.lim_for_gamma(self.gamma() + delta);

        if limit.is_normal() {
            return Err(lib_error("The delta given is not valid"))
        }

        if delta > Delta::ZERO {
            self.set_dir(Direction::CW);
        } else if delta < Delta::ZERO {
            self.set_dir(Direction::CCW);
        }

        Ok(())
    }

    fn drive_simple(&mut self, delta : Delta, speed_f : f32) -> Result<Delta, crate::Error> {
        if (1.0 < speed_f) | (0.0 > speed_f) {
            panic!("Invalid speed factor! {}", speed_f)
        }

        if !delta.is_normal() {
            return Ok(Delta::ZERO);
        }
        
        self.setup_drive(delta)?;

        let cur = self.create_builder(Omega::ZERO, self.omega_max() * speed_f);
        self.drive_curve(cur);

        Ok(delta)
    }

    fn drive_simple_int(&mut self, delta : Delta, speed_f : f32, intr : Interrupter, intr_data : &mut dyn MeasData) 
    -> Result<(Delta, bool), crate::Error> {
        if (1.0 < speed_f) | (0.0 > speed_f) {
            panic!("Invalid speed factor! {}", speed_f)
        }

        if !delta.is_normal() {
            return Ok((Delta::ZERO, true));
        }
        
        self.setup_drive(delta)?;

        let cur = self.create_builder(Omega::ZERO, self.omega_max() * speed_f);
        let (steps, interrupted) = self.drive_curve_int(cur, intr, intr_data);

        if self.dir().as_bool() {
            Ok((steps as f32 * self.consts.step_ang(self.micro), interrupted))
        } else {
            Ok((steps as f32 * -self.consts.step_ang(self.micro), interrupted))
        }
    }
}

impl<C : Controller + Send> HRStepper<C> {
    /// Makes the component move a single step with the given `time`
    /// 
    /// # Error
    /// 
    /// Returns an error if `setup_drive()` fails
    pub fn step(&mut self, time : Time) -> Result<(), crate::Error> {
        let delta = if self.dir().as_bool() { 
            self.consts.step_ang(self.micro) 
        } else { 
            -self.consts.step_ang(self.micro) 
        };
        self.setup_drive(delta)?;

        self.use_ctrl(|ctrl| {
            ctrl.step(time);
        });

        Ok(())
    }

    /// Sets the driving direction of the component. Note that the direction of the motor depends on the connection of the cables.
    /// The "dir"-pin is directly set to the value given
    #[inline(always)]
    pub fn set_dir(&mut self, dir : Direction) {
        self.use_ctrl(|ctrl| {
            ctrl.set_dir(dir);
        }); 
    }
}

// Async helper functions
#[cfg(feature = "embed-thread")]
impl<C : Controller + Send> HRStepper<C> {
    fn clear_active_status(&mut self) {
        let recv : &Receiver<AsyncRes>;

        if let Some(_recv) = &self.receiver {
            recv = _recv;
        } else {
            return; // TODO: Maybe add proper error message? 
        }

        loop {
            if recv.try_recv().is_err() {
                break;
            }
        }

        self.active = false;
    }

    fn drive_curve_async(&mut self, curve : Vec<Time>, intr : bool, t_const : Option<Time>) -> Result<(), crate::Error> {
        self.clear_active_status();

        // println!(" => Curve: {}; Last: {:?}; t_const: {:?}", curve.len(), curve.last(), t_const);

        if let Some(sender) = &self.sender {
            self.active = true;
            sender.send(Some((curve, intr, t_const)))?; 
            Ok(())
        } else {
            Err(no_async())
        }
    }

    fn drive_simple_async(&mut self, delta : Delta, speed_f : f32, t_const : Option<Time>) -> Result<(), crate::Error> {
        if (1.0 < speed_f) | (0.0 > speed_f) {
            panic!("Invalid speed factor! {}", speed_f)
        }
        
        if !delta.is_normal() {
            return Ok(());
        }

        if self.sender.is_none() {
            return Err(no_async());
        }
        
        self.setup_drive(delta)?;

        let cur = self.create_builder(Omega::ZERO, self.omega_max() * speed_f);

        self.drive_curve_async(cur.collect::<Vec<Time>>(), false, t_const)
    }

    fn setup_async(&mut self) {
        let (sender_com, receiver_thr) : (Sender<Option<AsyncMsg>>, Receiver<Option<AsyncMsg>>) = channel();
        let (sender_thr, receiver_com) : (Sender<AsyncRes>, Receiver<AsyncRes>) = channel();

        let ctrl_mut = self.ctrl.clone();
        let pos = self.pos.clone();

        self.thr = Some(std::thread::spawn(move || {
            let mut curve : Vec<Time>;
            let mut intr : bool;
            let mut cont : Option<Time>;

            let mut msg_sent = false;

            let mut msg_opt : Option<_> = None;
            let mut msg : Option<AsyncMsg>;

            loop {
                if let Some(msg_r) = msg_opt { 
                    msg = msg_r;
                } else {
                    msg = match receiver_thr.recv() {
                        Ok(msg_opt) => msg_opt,
                        Err(_) => {
                            // println!("Error occured in thread! {}", err.to_string());   // TODO: Improve error message
                            break;
                        }
                    };
                }

                match msg {
                    Some((msg_curve, msg_intr, msg_cont)) => {
                        curve = msg_curve;
                        intr = msg_intr;
                        cont = msg_cont;
                    },
                    None => {
                        break;
                    }
                };

                let mut ctrl = ctrl_mut.lock().unwrap();
                let mut index : usize = 0;
                let curve_len = curve.len();

                loop {
                    if index < curve_len {
                        ctrl.step(curve[index]);
                        pos.lock().unwrap().add_assign(if ctrl.dir().as_bool() { 1 } else { -1 });
                        index += 1;

                        if intr {
                            match receiver_thr.try_recv() {
                                Ok(msg) => { 
                                    msg_opt = Some(msg); 
                                    break;
                                },
                                Err(_) => { }
                            };
                        }

                        if index == curve_len {
                            sender_thr.send(if ctrl.dir().as_bool() {
                                index as i64
                            } else {
                                -(index as i64)
                            }).unwrap();

                            msg_sent = true;
                        }
                    } else if let Some(t_cont) = cont {
                        if !msg_sent {
                            sender_thr.send(if ctrl.dir().as_bool() {
                                index as i64
                            } else {
                                -(index as i64)
                            }).unwrap();

                            msg_sent = true;
                        }

                        loop {
                            ctrl.step(t_cont);
                            pos.lock().unwrap().add_assign(if ctrl.dir().as_bool() { 1 } else { -1 });
                            index += 1;

                            match receiver_thr.try_recv() {
                                Ok(msg) => { 
                                    msg_opt = Some(msg); 
                                    break;
                                },
                                Err(_) => { }
                            };
                        }

                        break;
                    } else {
                        if !msg_sent {
                            sender_thr.send(if ctrl.dir().as_bool() {
                                index as i64
                            } else {
                                -(index as i64)
                            }).unwrap();
                        }

                        msg_opt = None;
                        msg_sent = false;

                        break;
                    }
                }
            };
        }));

        self.sender = Some(sender_com);
        self.receiver = Some(receiver_com);
    }
}

impl<C : Controller + Send> Setup for HRStepper<C> {
    fn setup(&mut self) -> Result<(), crate::Error> {
        if self.lk.u == 0.0 {
            return Err("Link the construction to vaild data! (`LinkedData` is invalid)".into());
        }

        self.omega_max = self.consts.omega_max(self.lk.u);

        #[cfg(feature = "std")]
        self.setup_async();

        Ok(())
    }
}

impl<C : Controller + Send> SyncComp for HRStepper<C> {
    // Data
        fn vars<'a>(&'a self) -> &'a CompVars {
            &self.vars
        }

        fn link<'a>(&'a self) -> &'a LinkedData {
            &self.lk
        }

        #[inline]
        fn write_link(&mut self, lk : LinkedData) {
            self.lk = lk;
        }  
    // 

    // Movement
        fn drive_rel(&mut self, delta : Delta, speed_f : f32) -> Result<Delta, crate::Error> {
            self.drive_simple(delta, speed_f)
        }

        fn drive_abs(&mut self, gamma : Gamma, speed_f : f32) -> Result<Delta, crate::Error> {
            let delta = gamma - self.gamma();
            self.drive_simple(delta, speed_f)
        }

        fn drive_rel_int(&mut self, delta : Delta, speed_f : f32, intr : Interrupter, intr_data : &mut dyn MeasData) 
        -> Result<(Delta, bool), crate::Error> {
            self.drive_simple_int(delta, speed_f, intr, intr_data)
        }
    // 

    // Async
        #[cfg(feature = "std")]
        fn drive_rel_async(&mut self, delta : Delta, speed_f : f32) -> Result<(), crate::Error> {
            self.drive_simple_async(delta, speed_f, None)
        }

        #[cfg(feature = "std")]
        fn drive_abs_async(&mut self, gamma : Gamma, speed_f : f32) -> Result<(), crate::Error> {
            let delta = gamma - self.gamma();
            self.drive_simple_async(delta, speed_f, None)
        }
        
        #[cfg(feature = "std")]
        fn await_inactive(&mut self) -> Result<Delta, crate::Error> {
            if self.receiver.is_none() {
                return Err(no_async());
            }

            if !self.active {
                return Err(not_active());
            }

            let delta = if let Some(recv) = &self.receiver {
                self.consts.ang_from_steps(recv.recv().unwrap(), self.micro)        // TODO: Remove unwrap
            } else {
                Delta::NAN
            };

            self.active = false;
            Ok(delta)
        }
    //

    // Position
        #[inline]
        fn gamma(&self) -> Gamma {
            #[cfg(feature = "std")] {
                return Gamma::ZERO + self.consts.ang_from_steps(self.pos.lock().unwrap().clone(), self.micro); 
            }

            #[cfg(not(feature = "std"))] {
                return Gamma::ZERO + self.consts.ang_from_steps(self.pos); 
            }
        }   

        #[inline]
        fn write_gamma(&mut self, pos : Gamma) {
            #[cfg(feature = "std")] {
                *self.pos.lock().unwrap().deref_mut() = self.consts.steps_from_ang(pos - Gamma::ZERO, self.micro);
            }

            #[cfg(not(feature = "std"))] {
                self.pos = self.consts.steps_from_ang(pos - Gamma::ZERO);
            }
        }

        #[inline]
        fn omega_max(&self) -> Omega {
            self.omega_max
        }

        fn set_omega_max(&mut self, omega_max : Omega) {
            if omega_max > self.consts.omega_max(self.lk.u) {
                #[cfg(feature = "std")]
                panic!("Maximum omega must not be greater than recommended! (Given: {}, Rec: {})", omega_max, self.consts.omega_max(self.lk.u));
            }

            self.omega_max = omega_max;
        }

        #[inline]
        fn set_limit(&mut self, min : Option<Gamma>, max : Option<Gamma>) {
            if min.is_some() {
                self.vars.lim.min = min;
            }

            if max.is_some() {
                self.vars.lim.max = max;
            }
        }

        #[inline]
        fn reset_limit(&mut self, min : Option<Gamma>, max : Option<Gamma>) {
            self.vars.lim.min = min;
            self.vars.lim.max = max;
        }

        fn lim_for_gamma(&self, gamma : Gamma) -> Delta {
            match self.vars.lim.min {
                Some(ang) => {
                    if gamma < ang {
                        gamma - ang
                    } else {
                        match self.vars.lim.max {
                            Some(ang) => {
                                if gamma > ang {
                                    gamma - ang
                                } else { Delta::ZERO }
                            },
                            None => Delta::ZERO
                        }
                    }
                },
                None => match self.vars.lim.max {
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
            #[cfg(feature = "std")] {
                *self.pos.lock().unwrap().deref_mut() = self.consts.steps_from_ang(set_gamma - Gamma::ZERO, self.micro);
            }

            #[cfg(not(feature = "std"))] {
                self.pos = self.consts.steps_from_ang(set_gamma - Gamma::ZERO);
            }
    
            self.set_limit(
                if self.dir().as_bool() { self.vars.lim.min } else { Some(set_gamma) },
                if self.dir().as_bool() { Some(set_gamma) } else { self.vars.lim.max }
            )
        }
    //

    // Loads
        #[inline(always)]
        fn apply_inertia(&mut self, j : Inertia) {
            self.vars.j_load = j;
        }

        #[inline(always)]
        fn apply_force(&mut self, t : Force) {
            if t >= self.consts.t_s {
                #[cfg(feature = "std")]
                println!("Load will not be applied! {}", t);
                return;
            }

            self.vars.t_load = t;
        }

        #[inline(always)]
        fn apply_bend_f(&mut self, f_bend : f32) {
            self.vars.bend_f = f_bend;
        }
    //
}

#[cfg(feature = "std")]
impl<C : Controller + Send> AsyncComp for HRStepper<C> {
    fn drive(&mut self, dir : Direction, mut speed_f : f32) -> Result<(), crate::Error> {
        if (0.0 > speed_f) | (1.0 < speed_f) {
            panic!("Bad speed_f! {}", speed_f);
        }

        if self.active {
            return Err(crate::lib_error("The component is already active!"));
        }

        let omega_max = self.omega_max();
        let omega_0 = omega_max * self.speed_f;
        let omega_tar = omega_max * speed_f;

        // println!(" => Building curve: o_max: {}, o_0: {}, o_tar: {}, spf_0: {}, spf: {}", 
        //     self.omega_max, omega_0, omega_tar, self.speed_f, speed_f);

        let mut builder = CtrlStepTimeBuilder::from_builder(
            self.create_builder(Omega::ZERO, self.omega_max() * speed_f)
        );

        let t_const = if omega_tar != Omega::ZERO {
            Some(self.consts.step_time(omega_tar, self.micro))
        } else {
            None
        }; 
        
        let curve; 
        let curve_sec; 

        if dir == self.dir() {
            curve = builder.to_speed(omega_tar)?;
            curve_sec = vec![];
        } else {
            curve = builder.to_speed(Omega::ZERO)?; 
            curve_sec = builder.to_speed(omega_tar)?;
        }

        drop(builder);

        if dir == self.dir() {
            self.drive_curve_async(curve, true, t_const)?;
        } else {
            self.drive_curve_async(curve, true, None)?;

            self.await_inactive()?;
            self.set_dir(dir);

            self.drive_curve_async(curve_sec, true, t_const)?;
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

impl<C : Controller + Send> StepperComp for HRStepper<C> {
    fn consts(&self) -> &StepperConst {
        &self.consts
    }

    fn micro(&self) -> u8 {
        self.micro
    }

    fn set_micro(&mut self, micro : u8) {
        if cfg!(feature = "std") {
            self.pos.lock().unwrap().mul_assign((micro / self.micro) as i64);
        }

        self.micro = micro;
    }

    fn drive_nodes(&mut self, delta : Delta, omega_0 : Omega, omega_tar : Omega, corr : &mut (Delta, Time)) -> Result<(), crate::Error> {
        self.setup_drive(delta)?;

        let mut builder = self.create_builder(omega_0, self.omega_max());
        let curve = builder.to_speed_lim(delta, omega_0, omega_tar, corr)?;

        // dbg!(self.consts.steps_from_ang(delta));
        // dbg!(2.0 * delta / (omega_0 + omega_tar));
        // dbg!(curve.len());a
        // dbg!(curve.iter().map(|x| x.0).sum::<f32>());

        drop(builder);

        self.drive_curve_async(curve, false, None)?;

        Ok(())
    }
}