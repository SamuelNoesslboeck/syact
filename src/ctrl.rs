#[cfg(feature = "std")]
use core::time::Duration;

#[cfg(feature = "std")]
use std::sync::{Arc, Mutex};
#[cfg(feature = "std")]
use std::thread::JoinHandle;
#[cfg(feature = "std")]
use std::sync::mpsc::{Receiver, Sender, channel};

use crate::SyncComp;
use crate::data::{LinkedData, StepperConst, CompVars}; 
use crate::math;
use crate::units::*;

// Submodules
pub mod des;

pub mod pin;

#[cfg(feature = "std")]
pub mod pwm;

#[cfg(feature = "std")]
pub mod servo;
// 

type AsyncMsg = Vec<Time>;
type AsyncRes = ();

type Interrupter = fn (&mut Pins) -> bool;

#[inline(always)]
fn no_async() -> crate::Error {
    crate::Error::new(std::io::ErrorKind::NotConnected, "Async has not been setup yet!")
}

#[inline(always)]
fn not_active() -> crate::Error {
    crate::Error::new(std::io::ErrorKind::PermissionDenied, "No movement has been started yet")
}

// Pin struct
#[derive(Debug)]
struct Pins {
    /// Pin for defining the direction
    pub dir : pin::SimOutPin,
    /// Pin for PWM Step pulses
    pub step : pin::SimOutPin,
    /// Measurement pin
    pub meas : Option<pin::SimInPin>
}

#[derive(Debug, Clone, Default)]
struct Limits {
    /// Limit for minimum angle/step count
    pub min : Option<Gamma>,  
    /// Limit for maximum angle/step count
    pub max : Option<Gamma>
}

/// StepperCtrl
#[derive(Debug)]
pub struct StepperCtrl {
    /// Stepper data
    consts : StepperConst,
    vars : CompVars,

    /// The current direction of the driver, the bool value is written to the `sys.dir` GPIO pin\
    /// DO NOT WRITE TO THIS VALUE! 
    dir : bool,
    /// The current absolute position since set to a value
    pos : i64,

    lk : LinkedData,

    limit : Limits,

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
    active : bool
}

// Inits
impl StepperCtrl {   
    pub fn new(consts : StepperConst, pin_dir : u8, pin_step : u8) -> Self {
        // Create pins if possible
        let sys_dir = pin::UniPin::new(pin_dir).unwrap().into_output(); // TODO: Handle errors

        let sys_step = pin::UniPin::new(pin_step).unwrap().into_output();

        let mut ctrl = StepperCtrl { 
            consts, 
            vars: CompVars::ZERO, 

            dir: true, 
            pos: 0,

            lk: LinkedData::EMPTY,

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

            limit: Default::default(),

            #[cfg(feature = "std")]
            thr: None,
            #[cfg(feature = "std")]
            sender: None,
            #[cfg(feature = "std")]
            receiver: None,

            #[cfg(feature = "std")]
            active: false
        };

        ctrl.set_dir(ctrl.dir);

        ctrl
    }

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
    #[cfg(feature = "std")]
    fn step_sig(time : Time, pins : &mut Pins) {
        let step_time_half : Duration = (time / 2.0).into();

        pins.step.set_high();
        spin_sleep::sleep(step_time_half);
        pins.step.set_low();
        spin_sleep::sleep(step_time_half);
    }  

    fn drive_curve_sig(cur : &[Time], pins : &mut Pins) {
        for point in cur {
            StepperCtrl::step_sig(*point, pins);
        }
    }

    #[inline(always)]
    #[cfg(not(feature = "std"))]
    fn step_sig(time : Time, pins : &mut Pins) {
        // TODO: Add delay handler
    }   

    fn drive_curve(&mut self, cur : &[Time]) {
        #[cfg(feature = "std")]
        let mut pins = self.sys.lock().unwrap();

        #[cfg(not(feature = "std"))]
        let mut pins = self.sys;

        StepperCtrl::drive_curve_sig(cur, &mut pins);
        self.pos += if self.dir { cur.len() as i64 } else { -(cur.len() as i64) };

    }

    fn drive_curve_int(&mut self, cur : &[Time], intr : Interrupter) -> (usize, bool) {
        #[cfg(feature = "std")]
        let mut pins = self.sys.lock().unwrap();

        #[cfg(not(feature = "std"))]
        let mut pins = self.sys;

        let mut trav = 0;

        for point in cur {
            if intr(&mut pins) {
                break;
            }
    
            StepperCtrl::step_sig(*point, &mut pins);
            self.pos += if self.dir { 1 } else { -1 };

            trav += 1;
        }

        ( trav, trav != cur.len() )
    }

    fn setup_drive(&mut self, delta : Delta) -> Result<(), crate::Error> {
        let limit = self.lim_for_gamma(self.gamma() + delta);

        if limit.is_normal() {
            #[cfg(feature = "std")]
            return Err(crate::Error::new(std::io::ErrorKind::InvalidInput, "The delta given is not valid"))
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

    // fn drive_complex(&mut self, delta : Delta, omega_0 : Omega, omega_end : Omega, omega_max : Omega) -> Result<Delta, crate::Error> {
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
            sender.send(Some(cur)).unwrap(); // TODO: remove unwrap
        }

        Ok(())
    }
}

impl StepperCtrl {
    pub fn step(&mut self, time : Time) -> Result<(), crate::Error> {
        let delta = if self.dir { self.consts.step_ang() } else { -self.consts.step_ang() };
        self.setup_drive(delta)?;

        #[cfg(feature = "std")]
        let mut pins = self.sys.lock().unwrap(); 
        #[cfg(not(feature = "std"))]
        let mut pins = self.sys;

        StepperCtrl::step_sig(time, &mut pins);

        Ok(())
    }

    #[inline(always)]
    pub fn set_dir(&mut self, dir : bool) {
        #[cfg(feature = "std")]
        let mut pins = self.sys.lock().unwrap();
        #[cfg(not(feature = "std"))]
        let mut pins = self.sys;

        StepperCtrl::dir_sig(&mut pins, dir);
        self.dir = dir;
    }

    // Debug
        #[cfg(feature = "std")]
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

impl crate::math::MathActor for StepperCtrl {
    #[inline]
    fn accel_dyn(&self, omega : Omega, _ : Gamma) -> Alpha {
        self.consts.alpha_max_dyn(math::load::torque_dyn(&self.consts, omega, self.lk.u), &self.vars)
    }
}

impl SyncComp for StepperCtrl {
    // Setup function
        fn setup(&mut self) { }

        #[cfg(feature = "std")]
        fn setup_async(&mut self) {
            let (sender_com, receiver_thr) : (Sender<Option<AsyncMsg>>, Receiver<Option<AsyncMsg>>) = channel();
            let (sender_thr, receiver_com) : (Sender<AsyncRes>, Receiver<AsyncRes>) = channel();

            let sys = self.sys.clone();

            self.thr = Some(std::thread::spawn(move || {
                loop {
                    match receiver_thr.recv() {
                        Ok(msg_opt) => 
                            match msg_opt {
                                Some(msg) => {
                                    let mut pins = sys.lock().unwrap();

                                    StepperCtrl::drive_curve_sig(&msg, &mut pins);

                                    sender_thr.send(()).unwrap();
                                },
                                None => {
                                    break;
                                }
                            },
                        Err(err) => {
                            println!("Error occured in thread! {}", err.to_string());
                        }
                    }
                };
            }));

            self.sender = Some(sender_com);
            self.receiver = Some(receiver_com);
        }

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
            let (delta, interrupted) = self.drive_simple_int(max_delta, omega, Self::__meas_helper)?;

            if !interrupted {
                #[cfg(feature = "std")]
                return Err(crate::Error::new(std::io::ErrorKind::TimedOut, 
                    "The measurement failed for the fiven maximum delta distance"));
            }

            self.set_end(set_gamma);

            Ok(delta)
        }
    // 

    // Async
        fn drive_rel_async(&mut self, delta : Delta, omega : Omega) -> Result<(), crate::Error> {
            self.drive_simple_async(delta, omega)
        }

        fn drive_abs_async(&mut self, gamma : Gamma, omega : Omega) -> Result<(), crate::Error> {
            let delta = gamma - self.gamma();
            self.drive_simple_async(delta, omega)
        }
        
        fn await_inactive(&mut self) -> Result<(), crate::Error> {
            if self.receiver.is_none() {
                return Err(no_async());
            }

            if !self.active {
                return Err(not_active());
            }

            if let Some(recv) = &self.receiver {
                recv.recv().unwrap(); // TODO: Remove unwrap
            }

            self.active = false;
            Ok(())
        }
    //

    // Position
        #[inline]
        fn gamma(&self) -> Gamma {
            Gamma::ZERO + self.consts.ang_from_steps_dir(self.pos)
        }   

        #[inline]
        fn write_gamma(&mut self, pos : Gamma) {
            self.pos = self.consts.steps_from_ang_dir(pos - Gamma::ZERO);
        }

        #[inline]
        fn set_limit(&mut self, min : Option<Gamma>, max : Option<Gamma>) {
            if min.is_some() {
                self.limit.min = min;
            }

            if max.is_some() {
                self.limit.max = max;
            }
        }

        #[inline]
        fn reset_limit(&mut self, min : Option<Gamma>, max : Option<Gamma>) {
            self.limit.min = min;
            self.limit.max = max;
        }

        fn lim_for_gamma(&self, gamma : Gamma) -> Delta {
            match self.limit.min {
                Some(ang) => {
                    if gamma < ang {
                        gamma - ang
                    } else {
                        match self.limit.max {
                            Some(ang) => {
                                if gamma > ang {
                                    gamma - ang
                                } else { Delta::ZERO }
                            },
                            None => Delta::ZERO
                        }
                    }
                },
                None => match self.limit.max {
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
            self.pos = self.consts.steps_from_ang_dir(set_gamma - Gamma::ZERO);
    
            self.set_limit(
                if self.dir { self.limit.min } else { Some(set_gamma) },
                if self.dir { Some(set_gamma) } else { self.limit.max }
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
            self.vars.t_load = t;
        }
    //
}

