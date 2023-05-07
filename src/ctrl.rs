#[cfg(feature = "std")]
use core::time::Duration;

#[cfg(feature = "std")]
use std::sync::{Arc, Mutex};
#[cfg(feature = "std")]
use std::thread::JoinHandle;
#[cfg(feature = "std")]
use std::sync::mpsc::{Receiver, Sender, channel};

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
pub mod des;

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

type Interrupter = fn (&mut Pins) -> bool;

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
    pub step : pin::UniOutPin,
    /// Measurement pin
    pub meas : Option<pin::UniInPin>
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

            lk: LinkedData { u: 0.0, s_f: 0.0 },

            #[cfg(feature = "std")]
            sys: Arc::new(Mutex::new(Pins {
                dir: sys_dir,
                step: sys_step,
                meas: None,
            })),
            
            #[cfg(not(feature = "std"))]
            sys: Pins {
                dir: sys_dir,
                step: sys_step,
                meas: None,
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

    // Misc
        /// Helper function for measurements with a single pin
        #[inline]
        fn __meas_helper(pins : &mut Pins) -> bool {
            match &mut pins.meas {
                Some(pin) => pin.is_high(), 
                None => false
            }
        }
    //
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

    fn drive_curve_int(&mut self, cur : &[Time], intr : Interrupter) -> (usize, bool) {
        #[cfg(feature = "std")]
        let mut pins = self.sys.lock().unwrap();

        #[cfg(not(feature = "std"))]
        let pins = &mut self.sys;

        let mut trav = 0;

        for point in cur {
            #[cfg(feature = "std")]
            if intr(&mut pins) {
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

    fn drive_simple(&mut self, delta : Delta, omega_max : Omega) -> Result<Delta, crate::Error> {
        if !delta.is_normal() {
            return Ok(Delta::ZERO);
        }
        
        self.setup_drive(delta)?;

        let cur = math::curve::create_simple_curve(&self.consts, &self.vars, &self.lk, delta, omega_max);
        self.drive_curve(&cur);

        Ok(delta)
    }

    fn drive_simple_int(&mut self, delta : Delta, omega_max : Omega, intr : Interrupter) -> Result<(Delta, bool), crate::Error> {
        if !delta.is_normal() {
            return Ok((Delta::ZERO, true));
        }
        
        self.setup_drive(delta)?;

        let cur = math::curve::create_simple_curve(&self.consts, &self.vars, &self.lk, delta, omega_max);
        let (steps, interrupted) = self.drive_curve_int(&cur, intr);

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

    fn drive_simple_async(&mut self, delta : Delta, omega_max : Omega) -> Result<(), crate::Error> {
        if !delta.is_normal() {
            return Ok(());
        }

        if self.sender.is_none() {
            return Err(no_async());
        }
        
        self.setup_drive(delta)?;
        self.clear_active_status();

        let cur = math::curve::create_simple_curve(&self.consts, &self.vars, &self.lk, delta, omega_max);

        if let Some(sender) = &self.sender {
            self.active = true;
            sender.send(Some((cur, self.dir, None))).unwrap(); // TODO: remove unwrap
            Ok(())
        } else {
            Err(no_async())
        }
    }
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

impl crate::meas::SimpleMeas for StepperCtrl {
    #[cfg(feature = "std")]
    fn init_meas(&mut self, pin_mes : u8) {
        self.sys.lock().unwrap().meas = Some(
            pin::UniPin::new(pin_mes).unwrap().into_input()     // TODO: Proper error message
        )
    }

    #[cfg(not(feature = "std"))]
    fn init_meas(&mut self, pin_mes : u8) {
        self.sys.meas = Some(
            pin::UniPin::new(pin_mes).unwrap().into_input()     // TODO: Proper error message
        )
    }
}

// impl crate::math::MathActor for StepperCtrl {
//     #[inline]
//     fn accel_dyn(&self, omega : Omega, _ : Gamma) -> Alpha {
//         self.consts.alpha_max_dyn(math::load::torque_dyn(&self.consts, omega, self.lk.u), &self.vars).unwrap()
//     }
// }

impl Setup for StepperCtrl {
    fn setup(&mut self) -> Result<(), crate::Error> {
        Ok(())
    }
}

impl SyncComp for StepperCtrl {
    // Setup functions
        #[cfg(feature = "std")]
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

                    sender_thr.send(if dir {
                        index as i64
                    } else {
                        -(index as i64)
                    }).unwrap();
                };
            }));

            self.sender = Some(sender_com);
            self.receiver = Some(receiver_com);
        }

    // Data
        fn consts<'a>(&'a self) -> &'a StepperConst {
            &self.consts    
        }

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

    // JSON 
        fn to_json(&self) -> Result<serde_json::Value, serde_json::Error> {
            serde_json::to_value(self)
        }
    //

    // Movement
        fn drive_rel(&mut self, delta : Delta, omega : Omega) -> Result<Delta, crate::Error> {
            self.drive_simple(delta, omega)
        }

        fn drive_abs(&mut self, gamma : Gamma, omega : Omega) -> Result<Delta, crate::Error> {
            let delta = gamma - self.gamma();
            self.drive_simple(delta, omega)
        }

        fn measure(&mut self, max_delta : Delta, omega : Omega, set_gamma : Gamma) -> Result<Delta, crate::Error> {
            self.reset_limit(None, None);

            let (delta, interrupted) = self.drive_simple_int(max_delta, omega, Self::__meas_helper)?;

            if !interrupted {
                #[cfg(feature = "std")]
                return Err(lib_error("The measurement failed for the fiven maximum delta distance"));
            }

            self.set_end(set_gamma);

            Ok(delta)
        }
    // 

    // Async
        #[cfg(feature = "std")]
        fn drive_rel_async(&mut self, delta : Delta, omega : Omega) -> Result<(), crate::Error> {
            self.drive_simple_async(delta, omega)
        }

        #[cfg(feature = "std")]
        fn drive_abs_async(&mut self, gamma : Gamma, omega : Omega) -> Result<(), crate::Error> {
            let delta = gamma - self.gamma();
            self.drive_simple_async(delta, omega)
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
        if self.active {
            return Err(crate::lib_error("The component is already active!"));
        }

        self.set_dir(
            match dir {
                Direction::CW => true,
                Direction::CCW => false,
                Direction::None => { 
                    speed_f = 0.0; 
                    self.dir
                }
            }
        );

        let omega_max = self.consts.max_speed(self.lk.u);
        let omega_0 = omega_max * self.speed_f;
        let omega_tar = omega_max * speed_f;

        let mut builder = CurveBuilder::new(&self.consts, &self.vars, &self.lk, omega_0);

        let curve = builder.to_speed(omega_tar)?;

        if let Some(sender) = &mut self.sender {
            sender.send(Some((curve, self.dir, Some(1.0 / omega_tar)))).unwrap();
        } else {
            return Err(crate::lib_error("Async-thread is not setup yet! Call 'setup_async()' first!"));
        };

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