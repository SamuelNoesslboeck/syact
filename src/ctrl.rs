#[cfg(feature = "std")]
use core::time::Duration;

#[cfg(feature = "std")]
use std::sync::{Arc, Mutex};
#[cfg(feature = "std")]
use std::thread::JoinHandle;
#[cfg(feature = "std")]
use std::sync::mpsc::{Receiver, Sender, channel};

use crate::comp::stepper::StepperComp;
use crate::meas::MeasData;
use crate::{SyncComp, Setup, lib_error};
use crate::data::{LinkedData, StepperConst, CompVars}; 
use crate::math::{self, CurveBuilder};
use crate::units::*;

#[cfg(feature = "std")]
use crate::comp::asyn::{AsyncComp, Direction};

// Submodules
/// Basic DC-Motors
pub mod dc_motor;
pub use dc_motor::DcMotor;

/// Helper functions and structs for deserializing
/// 
/// # Features
/// 
/// Only available if the "std"-feature is available
#[cfg(feature = "std")]
mod des;

/// Universal pin structure
pub mod pin;

/// PWM-signal 
/// 
/// # Features
/// 
/// Only available if the "std"-feature is available
#[cfg(feature = "std")]
pub mod pwm;

/// Structs and methods for basic servo motors
/// 
/// # Features 
/// 
/// Only available if the "std"-feature is available
#[cfg(feature = "std")]
pub mod servo;
// 

#[cfg(feature = "std")]
type AsyncMsg = (Vec<Time>, bool, Option<Time>);
#[cfg(feature = "std")]
type AsyncRes = i64;

/// A function that can be used to interrupt the movement process of a component
pub type Interrupter<'a> = fn (&mut dyn MeasData) -> bool;

#[cfg(feature = "std")]
#[inline(always)]
fn no_async() -> crate::Error {
    lib_error("Async has not been setup yet!")
}

#[inline(always)]
#[cfg(feature = "std")]
fn not_active() -> crate::Error {
    lib_error("No movement has been started yet")
}

// Pin struct
#[derive(Debug)]
struct Pins {
    /// Pin for defining the direction
    pub dir : pin::UniOutPin,
    /// Pin for PWM Step pulses
    pub step : pin::UniOutPin
}

/// StepperCtrl
#[derive(Debug)]
pub struct StepperCtrl {
    // Stepper data
    consts : StepperConst,
    vars : CompVars,

    /// The current direction of the driver, the bool value is written to the `sys.dir` GPIO pin\
    /// DO NOT WRITE TO THIS VALUE! 
    dir : bool,
    /// The current absolute position since set to a value
    pos : i64,
    omega_max : Omega,

    lk : LinkedData,

    #[cfg(feature = "std")]
    sys : Arc<Mutex<Pins>>,

    #[cfg(not(feature = "std"))]
    sys : Pins,

    // Threading
    #[cfg(feature = "std")]
    thr : Option<JoinHandle<()>>,
    #[cfg(feature = "std")]
    sender : Option<Sender<Option<AsyncMsg>>>,
    #[cfg(feature = "std")]
    receiver : Option<Receiver<AsyncRes>>,

    #[cfg(feature = "std")]
    active : bool,
    
    #[cfg(feature = "std")]
    speed_f : f32
}

// Inits
impl StepperCtrl {   
    /// Creates a new stepper controller with the given stepper motor constants `consts`
    pub fn new(consts : StepperConst, pin_dir : u8, pin_step : u8) -> Self {
        // Create pins if possible
        let sys_dir = pin::UniPin::new(pin_dir).unwrap().into_output(); // TODO: Handle errors

        let sys_step = pin::UniPin::new(pin_step).unwrap().into_output();

        let mut ctrl = StepperCtrl { 
            consts, 
            vars: CompVars::ZERO, 

            dir: true, 
            pos: 0,
            omega_max: Omega::ZERO,

            lk: LinkedData { u: 0.0, s_f: 0.0 },

            #[cfg(feature = "std")]
            sys: Arc::new(Mutex::new(Pins {
                dir: sys_dir,
                step: sys_step
            })),
            
            #[cfg(not(feature = "std"))]
            sys: Pins {
                dir: sys_dir,
                step: sys_step
            },

            #[cfg(feature = "std")]
            thr: None,
            #[cfg(feature = "std")]
            sender: None,
            #[cfg(feature = "std")]
            receiver: None,

            #[cfg(feature = "std")]
            active: false,

            #[cfg(feature = "std")]
            speed_f: 0.0
        };

        ctrl.set_dir(ctrl.dir);

        ctrl
    }

    /// Creates a new structure with both pins set to [pin::ERR_PIN] just for simulation purposes
    #[inline]
    pub fn new_sim(data : StepperConst) -> Self {
        Self::new(data, pin::ERR_PIN, pin::ERR_PIN)
    }
}

impl StepperCtrl {
    #[inline(always)]
    fn dir_sig(pins : &mut Pins, dir : bool) {
        if dir {
            pins.dir.set_high();
        } else {
            pins.dir.set_low();
        }
    }

    #[inline(always)]
    // #[cfg()]
    fn step_sig(time : Time, pins : &mut Pins) {
        let step_time_half : Duration = (time / 2.0).into();

        pins.step.set_high();
        spin_sleep::sleep(step_time_half);
        pins.step.set_low();
        spin_sleep::sleep(step_time_half);
    }  

    // #[inline(always)]
    // #[cfg(not(feature = "std"))]
    // #[cfg(feature = "embedded")]
    // fn step_sig(time : Time, pins : &mut Pins) {
    //     // TODO: Add delay handler
    // }   


    fn drive_curve_sig(cur : &[Time], pins : &mut Pins) {
        for point in cur {
            StepperCtrl::step_sig(*point, pins);
        }
    }

    fn drive_curve(&mut self, cur : &[Time]) {
        #[cfg(feature = "std")]
        let mut pins = self.sys.lock().unwrap();

        #[cfg(not(feature = "std"))]
        let pins = &mut self.sys;

        #[cfg(feature = "std")]
        StepperCtrl::drive_curve_sig(cur, &mut pins);

        #[cfg(not(feature = "std"))]
        StepperCtrl::drive_curve_sig(cur, pins);

        self.pos += if self.dir { 
            cur.len() as i64 
        } else { 
            -(cur.len() as i64) 
        };
    }

    fn drive_curve_int(&mut self, cur : &[Time], intr : Interrupter, intr_data : &mut dyn MeasData) -> (usize, bool) {
        #[cfg(feature = "std")]
        let mut pins = self.sys.lock().unwrap();

        #[cfg(not(feature = "std"))]
        let pins = &mut self.sys;

        let mut trav = 0;

        for point in cur {
            #[cfg(feature = "std")]
            if intr(intr_data) {
                break;
            }

            #[cfg(not(feature = "std"))]
            if intr(pins) {
                break;
            }
    
            #[cfg(feature = "std")]
            StepperCtrl::step_sig(*point, &mut pins);

            #[cfg(not(feature = "std"))]
            StepperCtrl::step_sig(*point, pins);

            self.pos += if self.dir { 1 } else { -1 };

            trav += 1;
        }

        ( trav, trav != cur.len() )
    }

    fn setup_drive(&mut self, delta : Delta) -> Result<(), crate::Error> {
        let limit = self.lim_for_gamma(self.gamma() + delta);

        if limit.is_normal() {
            #[cfg(feature = "std")]
            return Err(lib_error("The delta given is not valid"))
        }

        if delta > Delta::ZERO {
            self.set_dir(true);
        } else if delta < Delta::ZERO {
            self.set_dir(false);
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

        let omega_max = self.omega_max() * speed_f;

        let cur = math::curve::create_simple_curve(&self.consts, &self.vars, &self.lk, delta, omega_max);
        self.drive_curve(&cur);

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

        let cur = math::curve::create_simple_curve(&self.consts, &self.vars, &self.lk, delta, self.omega_max() * speed_f);
        let (steps, interrupted) = self.drive_curve_int(&cur, intr, intr_data);

        if self.dir {
            Ok((steps as f32 * self.consts.step_ang(), interrupted))
        } else {
            Ok((steps as f32 * -self.consts.step_ang(), interrupted))
        }
    }

    // fn drive_complex(&mut self, delta : Delta, omega_0 : Omega, omega_end : Omega, omega_max : Omega) 
    //          -> Result<Delta, crate::Error> {
    //     if !delta.is_normal() {
    //         return Ok(Delta::ZERO);
    //     }

    //     self.setup_drive(delta)?;

    //     todo!();
    // }
}

impl StepperCtrl {
    /// Makes the component move a single step with the given `time`
    /// 
    /// # Error
    /// 
    /// Returns an error if `setup_drive()` fails
    pub fn step(&mut self, time : Time) -> Result<(), crate::Error> {
        let delta = if self.dir { self.consts.step_ang() } else { -self.consts.step_ang() };
        self.setup_drive(delta)?;

        #[cfg(feature = "std")]
        let mut pins = self.sys.lock().unwrap(); 
        #[cfg(not(feature = "std"))]
        let pins = &mut self.sys;

        #[cfg(feature = "std")]
        StepperCtrl::step_sig(time, &mut pins);
        #[cfg(not(feature = "std"))]
        StepperCtrl::step_sig(time, pins);

        Ok(())
    }

    /// Sets the driving direction of the component. Note that the direction of the motor depends on the connection of the cables.
    /// The "dir"-pin is directly set to the value given
    #[inline(always)]
    pub fn set_dir(&mut self, dir : bool) {
        #[cfg(feature = "std")]
        let mut pins = self.sys.lock().unwrap();
        #[cfg(not(feature = "std"))]
        let pins = &mut self.sys;

        #[cfg(feature = "std")]
        StepperCtrl::dir_sig(&mut pins, dir);
        #[cfg(not(feature = "std"))]
        StepperCtrl::dir_sig(pins, dir);

        self.dir = dir;
    }

    // Debug
        /// Prints out all pins to the console in dbg! style
        /// 
        /// # Features
        /// 
        /// Only available if the "std"-feature is active
        #[cfg(feature = "std")]
        #[inline(always)]
        pub fn debug_pins(&self) {
            dbg!(&self.sys);
        }
    // 
}

impl Setup for StepperCtrl {
    fn setup(&mut self) -> Result<(), crate::Error> {
        if self.lk.u == 0.0 {
            return Err("Link the construction to vaild data! (`LinkedData` is invalid)".into());
        }

        self.omega_max = self.consts.max_speed(self.lk.u);

        #[cfg(feature = "std")]
        self.setup_async();

        Ok(())
    }
}

#[cfg(feature = "std")]
impl StepperCtrl {
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

    fn drive_curve_async(&mut self, curve : Vec<Time>, t_const : Option<Time>) -> Result<(), crate::Error> {
        self.clear_active_status();

        println!(" => Curve: {}; Last: {:?}; t_const: {:?}", curve.len(), curve.last(), t_const);

        if let Some(sender) = &self.sender {
            self.active = true;
            sender.send(Some((curve, self.dir, t_const)))?; 
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

        let omega_max = self.omega_max() * speed_f;
        let cur = math::curve::create_simple_curve(&self.consts, &self.vars, &self.lk, delta, omega_max);

        self.drive_curve_async(cur, t_const)
    }

    fn setup_async(&mut self) {
        let (sender_com, receiver_thr) : (Sender<Option<AsyncMsg>>, Receiver<Option<AsyncMsg>>) = channel();
        let (sender_thr, receiver_com) : (Sender<AsyncRes>, Receiver<AsyncRes>) = channel();

        let sys = self.sys.clone();

        self.thr = Some(std::thread::spawn(move || {
            let mut curve : Vec<Time>;
            let mut dir : bool;
            let mut cont : Option<Time>;

            let mut msg_opt : Option<_> = None;
            let mut msg : Option<AsyncMsg>;

            loop {
                if let Some(msg_r) = msg_opt { 
                    msg = msg_r;
                } else {
                    msg = match receiver_thr.recv() {
                        Ok(msg_opt) => msg_opt,
                        Err(err) => {
                            println!("Error occured in thread! {}", err.to_string());   // TODO: Improve error message
                            break;
                        }
                    };
                }

                match msg {
                    Some((msg_curve, msg_dir, msg_cont)) => {
                        let steps_dir = if msg_dir {
                            msg_curve.len() as i64
                        } else {
                            -(msg_curve.len() as i64)
                        };

                        curve = msg_curve;
                        dir = msg_dir;
                        cont = msg_cont;

                        sender_thr.send(steps_dir).unwrap();
                    },
                    None => {
                        break;
                    }
                };

                let mut pins = sys.lock().unwrap();
                let mut index : usize = 0;
                let curve_len = curve.len();

                loop {
                    if index < curve_len {
                        Self::step_sig(curve[index], &mut pins);
                        index += 1;

                        match receiver_thr.try_recv() {
                            Ok(msg) => { 
                                msg_opt = Some(msg); 
                                break;
                            },
                            Err(_) => { }
                        };

                        if index == curve_len {
                            sender_thr.send(if dir {
                                index as i64
                            } else {
                                -(index as i64)
                            }).unwrap();
                        }
                    } else if let Some(t_cont) = cont {
                        loop {
                            Self::step_sig(t_cont, &mut pins);
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
                        msg_opt = None;

                        break;
                    }
                }
            };
        }));

        self.sender = Some(sender_com);
        self.receiver = Some(receiver_com);
    }
}

impl SyncComp for StepperCtrl {
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
                self.consts.ang_from_steps(recv.recv().unwrap())        // TODO: Remove unwrap
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
            Gamma::ZERO + self.consts.ang_from_steps(self.pos)
        }   

        #[inline]
        fn write_gamma(&mut self, pos : Gamma) {
            self.pos = self.consts.steps_from_ang(pos - Gamma::ZERO);
        }

        fn omega_max(&self) -> Omega {
            self.omega_max
        }

        fn set_omega_max(&mut self, omega_max : Omega) {
            if omega_max > self.consts.max_speed(self.lk.u) {
                #[cfg(feature = "std")]
                panic!("Maximum omega must not be greater than recommended! (Given: {}, Rec: {})", omega_max, self.consts.max_speed(self.lk.u));
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
            self.pos = self.consts.steps_from_ang(set_gamma - Gamma::ZERO);
    
            self.set_limit(
                if self.dir { self.vars.lim.min } else { Some(set_gamma) },
                if self.dir { Some(set_gamma) } else { self.vars.lim.max }
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
            self.vars.f_bend = f_bend;
        }
    //
}

#[cfg(feature = "std")]
impl AsyncComp for StepperCtrl {
    fn drive(&mut self, dir : Direction, mut speed_f : f32) -> Result<(), crate::Error> {
        if (0.0 > speed_f) | (1.0 < speed_f) {
            panic!("Bad speed_f! {}", speed_f);
        }

        if self.active {
            return Err(crate::lib_error("The component is already active!"));
        }

        let bdir = match dir {
            Direction::CW => true,
            Direction::CCW => false,
            Direction::None => { 
                speed_f = 0.0; 
                self.dir
            }
        };

        let omega_max = self.omega_max();
        let omega_0 = omega_max * self.speed_f;
        let omega_tar = omega_max * speed_f;

        // println!(" => Building curve: o_max: {}, o_0: {}, o_tar: {}, spf_0: {}, spf: {}", 
        //     self.omega_max, omega_0, omega_tar, self.speed_f, speed_f);

        let mut builder = CurveBuilder::new(&self.consts, &self.vars, &self.lk, omega_0);
        let t_const = if omega_tar != Omega::ZERO {
            Some(self.consts.step_time(omega_tar))
        } else {
            None
        }; 
        
        let curve; 
        let curve_sec; 

        if bdir == self.dir {
            curve = builder.to_speed(omega_tar)?;
            curve_sec = vec![];
        } else {
            curve = builder.to_speed(Omega::ZERO)?; 
            curve_sec = builder.to_speed(omega_tar)?;
        }

        drop(builder);

        if bdir == self.dir {
            self.drive_curve_async(curve, t_const)?;
        } else {
            self.drive_curve_async(curve, None)?;

            self.await_inactive()?;
            self.set_dir(bdir);

            self.drive_curve_async(curve_sec, t_const)?;
        }

        self.speed_f = speed_f;

        Ok(())
    }

    fn dir(&self) -> Direction {
        if !self.speed_f.is_normal() {
            Direction::None
        } else if self.dir {
            Direction::CW 
        } else {
            Direction::CCW
        }
    }

    fn speed_f(&self) -> f32 {
        self.speed_f
    }
}

impl StepperComp for StepperCtrl {
    fn consts(&self) -> &StepperConst {
        &self.consts
    }
}