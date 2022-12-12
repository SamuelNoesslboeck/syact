use std::{
    sync::{
        mpsc::{channel, Sender, Receiver}, 
        Mutex,
        Arc
    },
    thread::{self, JoinHandle}, 
    time, 
    vec
};

use gpio::{GpioIn, GpioOut, sysfs::*};

use crate::data::StepperData;
use crate::math::{start_frequency, angluar_velocity_dyn};

// Use local types module
mod types;
pub use types::*;

pub struct Driver 
{
    /// Stepper data
    pub data : StepperData,

    /// The current direction (true for right, false for left)
    pub dir : bool,
    /// The current absolute position since set to a value
    pub pos : i64,

    /// Pin for defining the direction
    sys_dir : RaspPin,
    /// Pin for PWM Step pulses
    sys_step : RaspPin,
    sys_meas : RaspPin,

    limit_min : LimitType,
    limit_max : LimitType
}

/// StepperCtrl
pub struct StepperCtrl
{
    /// Motor driver
    pub driver : Arc<Mutex<Driver>>,
    /// Async comms
    pub comms : StepperComms,

    /// Pin for controlling direction
    pub pin_dir : u16,
    /// Pin for controlling steps
    pub pin_step : u16,
    /// Pin for messuring distances
    pub pin_mes : u16, 
}

impl Driver {
    // Init
    pub fn new(data : StepperData, pin_dir : u16, pin_step : u16) -> Self {
        // Create pins if possible
        let sys_dir = match SysFsGpioOutput::open(pin_dir.clone()) {
            Ok(val) => RaspPin::Output(val),
            Err(_) => RaspPin::ErrPin
        };

        let sys_step = match SysFsGpioOutput::open(pin_step.clone()) {
            Ok(val) => RaspPin::Output(val),
            Err(_) => RaspPin::ErrPin
        };

        let mut driver = Driver {
            data, 
            dir: true, 
            pos: 0,
            
            sys_dir,
            sys_step,
            sys_meas: RaspPin::ErrPin,

            limit_min: LimitType::None,
            limit_max: LimitType::None
        };

        driver.set_dir(driver.dir);

        driver
    }


    // Misc
    fn __meas_helper(pin : &mut RaspPin) -> bool {
        match pin {
            RaspPin::Input(gpio_pin) => {
                gpio_pin.read_value().unwrap() == gpio::GpioValue::High
            },
            _ => true
        }
    }

    // Move
    pub fn step(&mut self, time : f32, ufunc : &UpdateFunc) -> StepResult {
        let step_time_half = time::Duration::from_secs_f32(time / 2.0);
        
        match &mut self.sys_step {
            RaspPin::Output(pin) => {
                pin.set_high().unwrap();
                thread::sleep(step_time_half);
                pin.set_low().unwrap();
                thread::sleep(step_time_half);
                
        
                self.pos += if self.dir { 1 } else { -1 };

                match self.limit_max {
                    LimitType::Angle(pos) => {
                        if self.get_abs_pos() > pos {
                            return StepResult::Break;
                        }
                    }, 
                    _ => { }
                };

                match self.limit_min {
                    LimitType::Angle(pos) => {
                        if self.get_abs_pos() < pos {
                            return StepResult::Break;
                        }
                    }, 
                    _ => { }
                };

                return match ufunc {
                    UpdateFunc::Break(func, steps) => {
                        if (self.pos % (*steps as i64)) == 0 {
                            if func(&mut self.sys_meas) {
                                println!("Meas conducted!");
                                return StepResult::Break
                            } 
                        }

                        StepResult::None
                    },
                    _ => StepResult::None
                }
            },
            _ => StepResult::Error
        }
    }

    pub fn accelerate(&mut self, stepcount : u64, omega : f32, ufunc : &UpdateFunc) -> (StepResult, Vec<f32>) {
        let t_start = self.data.sf / start_frequency(&self.data);
        let t_min = self.data.time_step(omega);

        let mut o_last : f32 = 0.0;
        let mut t_total : f32 = t_start;
        let mut time_step : f32 = t_start;          // Time per step
        let mut i: u64 = 1;                         // Step count
        let mut curve = vec![];

        self.step(time_step, ufunc);
        curve.push(time_step);

        loop {
            if i >= stepcount {
                break;
            }

            o_last = angluar_velocity_dyn(&self.data, t_total, o_last);
            time_step = self.data.step_ang() / o_last * self.data.sf;
            t_total += time_step;

            if time_step < t_min {
                break;
            }

            match self.step(time_step, ufunc) {
                StepResult::Break => {
                    return ( StepResult::Break, curve )
                },
                _ => { }
            }

            curve.push(time_step);
            i += 1;
        }

        ( StepResult::None, curve )
    }

    pub fn drive_curve(&mut self, curve : &Vec<f32>) {
        for i in 0 .. curve.len() {
            self.step(curve[i], &UpdateFunc::None);
        }
    }

    pub fn steps(&mut self, stepcount : u64, omega : f32, ufunc : UpdateFunc) -> StepResult {
        let ( result, mut curve ) = self.accelerate( stepcount / 2, omega, &ufunc);
        let time_step = self.data.time_step(omega);
        let last = curve.last().unwrap_or(&time_step);

        match result {
            StepResult::Break => {
                return result;
            },
            _ => { }
        }

        if (stepcount % 2) == 1 {
            self.step(*last, &UpdateFunc::None);
        }
        
        for _ in 0 .. 2 {
            for _ in curve.len() .. (stepcount / 2) as usize {
                match self.step(time_step, &ufunc) {
                    StepResult::Break => {
                        return StepResult::Break;
                    },
                    _ => { }
                }
            }
        }

        curve.reverse();
        self.drive_curve(&curve);

        StepResult::None
    }

    pub fn drive(&mut self, distance : f32, omega : f32, ufunc : UpdateFunc) -> f32 {
        if distance == 0.0 {
            return 0.0;
        } else if distance > 0.0 {
            self.set_dir(true);
        } else if distance < 0.0 {
            self.set_dir(false);
        }

        let steps : u64 = self.data.ang_to_steps(distance);
        self.steps(steps, omega, ufunc);
        return steps as f32 * self.data.step_ang();
    }

    pub fn set_dir(&mut self, dir : bool) {
        match &mut self.sys_dir {
            RaspPin::Output(pin) => {
                self.dir = dir;
                
                if self.dir {
                    pin.set_high().unwrap();
                } else {
                    pin.set_low().unwrap();
                }
            },
            _ => { }
        };
    }

    // Position
        pub fn get_abs_pos(&self) -> f32 {
            self.steps_to_ang_dir(self.pos)
        }

        pub fn write_pos(&mut self, pos : f32) {
            self.pos = self.ang_to_steps_dir(pos);
        }
    //

    // Limits
        pub fn set_limit(&mut self, min : LimitType, max : LimitType) {
            self.limit_min = min;
            self.limit_max = max;
        }

        pub fn get_limit_dest(&self, pos : f32) -> LimitDest {
            let res_min = match self.limit_min {
                LimitType::Angle(ang) => {
                    if pos < ang {
                        return LimitDest::Minimum(pos - ang);
                    }

                    LimitDest::NotReached
                },
                _ => LimitDest::NoLimitSet
            };
            
            if !res_min.reached() {
                return match self.limit_max {
                    LimitType::Angle(ang) => {
                        if pos > ang {
                            return LimitDest::Maximum(pos - ang);
                        }
    
                        LimitDest::NotReached
                    },
                    _ => res_min
                };
            } else {
                return res_min;
            }
        }

        pub fn set_endpoint(&mut self, set_pos : f32) {
            if Self::__meas_helper(&mut self.sys_meas) {
                self.pos = self.ang_to_steps_dir(set_pos);
    
                self.set_limit(
                    if self.dir { self.limit_min.clone() } else { LimitType::Angle(set_pos) },
                    if self.dir { LimitType::Angle(set_pos) } else { self.limit_max.clone() }
                )
            }
        }
    //

    // Conversions
        pub fn ang_to_steps(&self, ang : f32) -> u64 {
            (ang.abs() / self.data.step_ang()).round() as u64
        }

        pub fn ang_to_steps_dir(&self, ang : f32) -> i64 {
            (ang / self.data.step_ang()).round() as i64
        }

        pub fn steps_to_ang(&self, steps : u64) -> f32 {
            steps as f32 * self.data.step_ang()
        }

        pub fn steps_to_ang_dir(&self, steps : i64) -> f32 {
            steps as f32 * self.data.step_ang()
        }
    //

    // Loads
        pub fn apply_load_j(&mut self, j : f32) {
            self.data.apply_load_j(j);
        }

        pub fn apply_load_t(&mut self, t : f32) {
            self.data.apply_load_t(t);
        }
    // 

    // Debug
        pub fn debug_pins(&self) {
            dbg!(
                &self.sys_dir,
                &self.sys_step,
                &self.sys_meas
            );
        }
    // 
}

impl StepperCtrl
{   
    pub fn new(data : StepperData, pin_dir : u16, pin_step : u16) -> Self {
        let driver = Arc::new(Mutex::new(Driver::new(data, pin_dir, pin_step)));

        let ctrl = StepperCtrl { 
            pin_dir: pin_dir, 
            pin_step: pin_step, 
            pin_mes: PIN_ERR, 

            comms: StepperComms::new(Arc::clone(&driver)),
            driver: driver
        };

        ctrl
    }

    // Init
        pub fn init_meas(&mut self, pin_mes : u16) {
            self.pin_mes = pin_mes;
            self.driver.lock().unwrap().sys_meas = match SysFsGpioInput::open(pin_mes) {
                Ok(val) => RaspPin::Input(val),
                Err(_) => RaspPin::ErrPin
            }
        }
    //

    // Movements
        pub fn step(&mut self, time : f32, ufunc : &UpdateFunc) -> StepResult {
            self.driver.lock().unwrap().step(time, ufunc)
        }

        pub fn accelerate(&mut self, stepcount : u64, omega : f32, ufunc : &UpdateFunc) -> (StepResult, Vec<f32>) {
            self.driver.lock().unwrap().accelerate(stepcount, omega, ufunc)
        }

        pub fn drive_curve(&mut self, curve : &Vec<f32>) {
            for i in 0 .. curve.len() {
                self.step(curve[i], &UpdateFunc::None);
            }
        }

        pub fn steps(&mut self, stepcount : u64, omega : f32, ufunc : UpdateFunc) -> StepResult {
            self.driver.lock().unwrap().steps(stepcount, omega, ufunc)
        }

        pub fn drive(&mut self, distance : f32, omega : f32, ufunc : UpdateFunc) -> f32 {
            self.driver.lock().unwrap().drive(distance, omega, ufunc)
        }

        pub fn drive_async(&mut self, dist : f32, omega : f32, ufunc : UpdateFunc) {
            self.comms.drive_async(dist, omega, ufunc);
        }
    //

    // Direction
        pub fn get_dir(&self) -> bool {
            self.driver.lock().unwrap().dir
        }
        
        pub fn set_dir(&mut self, dir : bool) {
            self.driver.lock().unwrap().set_dir(dir);
        }
    //

    // Position
        pub fn get_abs_pos(&self) -> f32 {
            self.driver.lock().unwrap().get_abs_pos()
        }

        pub fn write_pos(&mut self, pos : f32) {
            self.driver.lock().unwrap().write_pos(pos);
        }
    //

    // Limits
        pub fn set_limit(&mut self, min : LimitType, max : LimitType) {
            self.driver.lock().unwrap().set_limit(min, max)
        }
    
        pub fn get_limit_dest(&self, pos : f32) -> LimitDest {
            self.driver.lock().unwrap().get_limit_dest(pos)
        }

        pub fn set_endpoint(&mut self, set_pos : f32) {
            self.driver.lock().unwrap().set_endpoint(set_pos)
        }
    // 

    pub fn measure(&mut self, max_pos : f32, omega : f32, set_pos : f32, accuracy : u64) {
        self.drive(max_pos, omega, UpdateFunc::Break(Driver::__meas_helper, accuracy));
        self.set_endpoint(set_pos);
    }

    pub fn measure_async(&mut self, max_dist : f32, omega : f32, set_pos : f32, accuracy : u64) {
        self.drive_async(max_dist, omega, UpdateFunc::Break(Driver::__meas_helper, accuracy));
        self.set_endpoint(set_pos);
    }

    // Loads
        pub fn apply_load_j(&mut self, j : f32) {
            self.driver.lock().unwrap().apply_load_j(j);
        }

        pub fn apply_load_t(&mut self, t : f32) {
            self.driver.lock().unwrap().apply_load_t(t);
        }
    // 

    // Debug 
        pub fn debug_pins(&self) {
            self.driver.lock().unwrap().debug_pins();
        }
    //
}

pub type Message = (f32, f32, UpdateFunc);
pub type Response = ();

pub struct StepperComms
{
    pub thr : JoinHandle<()>,
    pub sender : Sender<Message>,
    pub receiver : Receiver<Response>
}

impl StepperComms {
    pub fn new(ctrl : Arc<Mutex<Driver>>) -> Self {
        let (sender_com, receiver_thr) : (Sender<Message>, Receiver<Message>) = channel();
        let (sender_thr, receiver_com) : (Sender<Response>, Receiver<Response>) = channel();

        let thr = thread::spawn(move || {
            loop {
                let msg = receiver_thr.recv().unwrap();
                let mut sel = ctrl.lock().unwrap();
                
                sel.drive(msg.0, msg.1, msg.2);

                // drop(sel);

                sender_thr.send(()).unwrap();
            };
        });

        Self {
            thr, 
            sender: sender_com,
            receiver: receiver_com
        }
    }

    pub fn drive_async(&self, dist : f32, omega : f32, ufunc : UpdateFunc) {
        self.sender.send((dist, omega, ufunc)).unwrap();
    }

    pub fn await_inactive(&self) {
        self.receiver.recv().unwrap()
    }
}