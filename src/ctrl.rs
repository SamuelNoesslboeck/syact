use std::{
    sync::{
        mpsc::{channel, Sender, Receiver}, 
        Mutex, 
        Arc
    },
    thread::{self, JoinHandle}, 
    time::Duration, 
    vec
};

use gpio::{GpioIn, GpioOut, sysfs::*};

use crate::data::{StepperData, ServoData};
use crate::math::{start_frequency, angluar_velocity_dyn};

// Use local types module
mod types;
pub use types::*;

/// Trait for defining controls and components
pub trait Component 
{
    /// Move the component to the given position as fast as possible and returns the actual distance traveled
    ///  - The distance `dist` can be either an angle (Unit radians) or a distancce (Unit mm)
    ///  - The velocity `vel` is the maximum change rate of the distance, either angular velocity (Unit radians per secoond) or linear velocity (mm per second)
    fn drive(&mut self, dist : f32, vel : f32) -> f32;

    /// Move the component to the given position as fast as possible
    ///  - The distance `dist` can be either an angle (Unit radians) or a distancce (Unit mm)
    ///  - The velocity `vel` is the maximum change rate of the distance, either angular velocity (Unit radians per secoond) or linear velocity (mm per second) \
    /// To wait unti the movement operation is completed, use the `await inactive` function
    fn drive_async(&mut self, dist : f32, vel : f32);

    /// Measure the component by driving the component with the velocity `vel` until either the measurement condition is true or the maximum distance `dist` 
    /// is reached. When the endpoint is reached, the controls will set the distance to `set_dist`. The lower the `accuracy`, the higher 
    /// are the computational difficulties, as the function checks more often if the measure pin has a HIGH signal
    fn measure(&mut self, dist : f32, vel : f32, set_dist : f32, accuracy : u64) -> bool;

    fn measure_async(&mut self, dist : f32, vel : f32, accuracy : u64);

    // Position
        fn get_dist(&self) -> f32;

        fn drive_pos(&mut self, dist : f32, vel : f32) -> f32;

        fn write_dist(&mut self, dist : f32);
    // 

    // Load calculation
        fn apply_load_force(&mut self, force : f32);

        fn apply_load_inertia(&mut self, inertia : f32);
    // 
}

/// ### Driver
/// Driver class for basic stepper motor operations
pub struct StepperDriver 
{
    /// Stepper data
    pub data : StepperData,

    /// The current direction of the driver, the bool value is written to the `pin_dir` GPIO pin\
    /// DO NOT WRITE TO THIS VALUE! Use the `Driver::set_dir()` function instead
    pub dir : bool,
    /// The current absolute position since set to a value
    pub pos : i64,

    /// Pin for defining the direction
    sys_dir : RaspPin,
    /// Pin for PWM Step pulses
    sys_step : RaspPin,
    /// Measurement pin
    sys_meas : RaspPin,

    /// Limit for minimum angle/step count
    limit_min : LimitType,
    /// Limit for maximum angle/step count
    limit_max : LimitType
}

/// StepperCtrl
pub struct StepperCtrl
{
    /// Motor driver
    pub driver : Arc<Mutex<StepperDriver>>,
    /// Async comms
    pub comms : AsyncStepper,

    /// Pin for controlling direction
    pub pin_dir : u16,
    /// Pin for controlling steps
    pub pin_step : u16,
    /// Pin for messuring distances
    pub pin_mes : u16, 
}

pub struct ServoDriver
{
    pub pos : f32,
    pub pin_pwm : u16,

    // Thread
    pub thr : JoinHandle<()>,
    pub sender : Sender<f32>
}

impl StepperDriver {
    // Init
    /// Creates a new Driver from the given stepper `StepperData` \
    /// Pin numbers are based on the BCM-Scheme, not by absolute board numbers
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

        let mut driver = StepperDriver {
            data, 
            dir: true, 
            pos: 0,
            
            sys_dir,
            sys_step,
            sys_meas: RaspPin::ErrPin,

            limit_min: LimitType::None,
            limit_max: LimitType::None
        };

        // Write initial direction to output pins
        driver.set_dir(driver.dir);

        driver
    }


    // Misc
        /// Helper function for measurements with a single pin
        fn __meas_helper(pin : &mut RaspPin) -> bool {
            match pin {
                RaspPin::Input(gpio_pin) => {
                    gpio_pin.read_value().unwrap() == gpio::GpioValue::High
                },
                _ => true
            }
        }
    //

    // Movements
        /// Move a single step into the previously set direction. Uses `thread::sleep()` for step times, so the function takes `time` in seconds to process
        pub fn step(&mut self, time : f32, ufunc : &UpdateFunc) -> StepResult {
            match &mut self.sys_step {
                RaspPin::Output(pin) => {
                    let step_time_half = Duration::from_secs_f32(time / 2.0);

                    pin.set_high().unwrap();
                    thread::sleep(step_time_half);
                    pin.set_low().unwrap();
                    thread::sleep(step_time_half);
                    
            
                    self.pos += if self.dir { 1 } else { -1 };
        
                    match self.limit_max {
                        LimitType::Angle(pos) => {
                            if self.get_dist() > pos {
                                return StepResult::Break;
                            }
                        }, 
                        _ => { }
                    };
        
                    match self.limit_min {
                        LimitType::Angle(pos) => {
                            if self.get_dist() < pos {
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
            let t_min = self.data.step_time(omega);

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
            let time_step = self.data.step_time(omega);
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

            let steps : u64 = self.data.ang_to_steps_dir(distance).abs() as u64;
            self.steps(steps, omega, ufunc);
            return steps as f32 * self.data.step_ang();
        }

        pub fn set_dir(&mut self, dir : bool) {
            // Added dir assignment for debug purposes
            self.dir = dir;

            match &mut self.sys_dir {
                RaspPin::Output(pin) => {
                    // Removed dir assignment from here
                    
                    if self.dir {
                        pin.set_high().unwrap();
                    } else {
                        pin.set_low().unwrap();
                    }
                },
                _ => { }
            };
        }
    // 

    // Position
        pub fn get_dist(&self) -> f32 {
            self.steps_to_ang_dir(self.pos)
        }

        pub fn write_dist(&mut self, pos : f32) {
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

        pub fn set_endpoint(&mut self, set_pos : f32) -> bool {
            if Self::__meas_helper(&mut self.sys_meas) {
                self.pos = self.ang_to_steps_dir(set_pos);
    
                self.set_limit(
                    if self.dir { self.limit_min.clone() } else { LimitType::Angle(set_pos) },
                    if self.dir { LimitType::Angle(set_pos) } else { self.limit_max.clone() }
                );

                true
            } else {
                false
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
        pub fn apply_load_inertia(&mut self, j : f32) {
            self.data.apply_load_j(j);
        }

        pub fn apply_load_force(&mut self, t : f32) {
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
        let driver = Arc::new(Mutex::new(StepperDriver::new(data, pin_dir, pin_step)));

        let ctrl = StepperCtrl { 
            pin_dir: pin_dir, 
            pin_step: pin_step, 
            pin_mes: PIN_ERR, 

            comms: AsyncStepper::new(Arc::clone(&driver), 
                |driver_mutex , msg| { 
                    let mut driver = driver_mutex.lock().unwrap();

                    println!("Proccessing msg in thread: {} {}", msg.0, msg.1); 
                    driver.drive(msg.0, msg.1, msg.2);

                    () 
                }),
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
    //

    // Direction
        pub fn get_dir(&self) -> bool {
            self.driver.lock().unwrap().dir
        }
        
        pub fn set_dir(&mut self, dir : bool) {
            self.driver.lock().unwrap().set_dir(dir);
        }
    //

    // Limits
        pub fn set_limit(&mut self, min : LimitType, max : LimitType) {
            self.driver.lock().unwrap().set_limit(min, max)
        }
    
        pub fn get_limit_dest(&self, pos : f32) -> LimitDest {
            self.driver.lock().unwrap().get_limit_dest(pos)
        }

        pub fn set_endpoint(&mut self, set_pos : f32) -> bool {
            self.driver.lock().unwrap().set_endpoint(set_pos)
        }
    // 

    // Debug 
        pub fn debug_pins(&self) {
            self.driver.lock().unwrap().debug_pins();
        }
    //
}

impl Component for StepperCtrl 
{
    fn drive(&mut self, distance : f32, omega : f32) -> f32 {
        self.driver.lock().unwrap().drive(distance, omega, UpdateFunc::None)
    }

    fn drive_async(&mut self, dist : f32, omega : f32) {
        self.comms.send_msg((dist, omega, UpdateFunc::None));
    }

    fn measure(&mut self, max_pos : f32, omega : f32, set_pos : f32, accuracy : u64) -> bool {
        let mut driver = self.driver.lock().unwrap();

        driver.drive(max_pos, omega, UpdateFunc::Break(StepperDriver::__meas_helper, accuracy));
        driver.set_endpoint(set_pos)
    }

    fn measure_async(&mut self, max_dist : f32, omega : f32, accuracy : u64) {
        self.comms.send_msg((max_dist, omega, UpdateFunc::Break(StepperDriver::__meas_helper, accuracy)));
    }

    // Position
        fn get_dist(&self) -> f32 {
            self.driver.lock().unwrap().get_dist()
        }

        fn drive_pos(&mut self, dist : f32, vel : f32) -> f32 {
            self.driver.lock().unwrap().drive(dist - self.get_dist(), vel, UpdateFunc::None)
        }

        fn write_dist(&mut self, pos : f32) {
            self.driver.lock().unwrap().write_dist(pos);
        }
    //

    // Loads
        fn apply_load_inertia(&mut self, inertia : f32) {
            self.driver.lock().unwrap().apply_load_inertia(inertia);
        }

        fn apply_load_force(&mut self, force : f32) {
            self.driver.lock().unwrap().apply_load_force(force);
        }
    //
}

impl ServoDriver 
{
    pub fn new(data : ServoData, pin_pwm : u16) -> Self {
        let pos = data.phi_max / 2.0;

        let mut sys_pwm = match SysFsGpioOutput::open(pin_pwm.clone()) {
            Ok(val) => RaspPin::Output(val),
            Err(_) => RaspPin::ErrPin
        };

        let (sender, recv) : (Sender<f32>, Receiver<f32>) = channel();

        let thr = thread::spawn(move || {
            let mut pos = data.default_pos();

            loop {
                match recv.try_recv() {
                    Ok(ang) => { pos = ang; },
                    _ => { }
                }

                ServoDriver::pulse(&data, &mut sys_pwm, pos);
            }
        }); 

        ServoDriver {
            pos,
            thr,
            sender,
            pin_pwm
        }
    }

    pub fn pulse(data : &ServoData, sys_pwm : &mut RaspPin, pos : f32) {
        match sys_pwm {
            RaspPin::Output(pin) => {
                let cycle = data.cycle_time();
                let pulse = data.pulse_time(pos);

                pin.set_high().unwrap();
                thread::sleep(Duration::from_secs_f32(pulse));
                pin.set_low().unwrap(); 
                thread::sleep(Duration::from_secs_f32(cycle - pulse));
            },
            _ => { }
        }
    }
}


type StepperMsg = (f32, f32, UpdateFunc);
type StepperRes = ();

type AsyncStepper = AsyncComms<StepperMsg, StepperRes>;

// type ServoMsg = f32;
// type ServoRes = ();

// type AsyncServo = AsyncComms<ServoMsg, ServoRes>;

type CommsFunc<Ctrl, Msg, Res> = fn (&mut Ctrl, Msg) -> Res;

pub struct AsyncComms<Msg: Send + 'static, Res: Send + 'static>
{
    pub thr : JoinHandle<()>,
    sender : Sender<Option<Msg>>,
    receiver : Receiver<Res>
}


/// ### `StepperComms`
/// Struct for managing async movements
/// ```rust
/// use stepper_lib::{StepperCtrl, StepperData, UpdateFunc};
/// use std::f32::consts::PI;
/// 
/// let ctrl = StepperCtrl::new(StepperData::mot_17he15_1504s(12.0, 1.5), 27, 19);
/// ctrl.comms.drive_async(4.0 * PI, 2.0 * PI, UpdateFunc::None);
///
/// ctrl.comms.await_inactive();
/// ```
impl<Msg: Send + 'static, Res: Send + 'static> AsyncComms<Msg, Res> {
    pub fn new<Ctrl: Send + 'static>(mut ctrl : Ctrl, comms_func : CommsFunc<Ctrl, Msg, Res>) -> Self {
        let (sender_com, receiver_thr) : (Sender<Option<Msg>>, Receiver<Option<Msg>>) = channel();
        let (sender_thr, receiver_com) : (Sender<Res>, Receiver<Res>) = channel();

        let thr = thread::spawn(move || {
            loop {
                match receiver_thr.recv() {
                    Ok(msg_opt) => 
                        match msg_opt {
                            Some(msg) => {
                                let res = comms_func(&mut ctrl, msg);
                
                                sender_thr.send(res).unwrap();
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
        });

        Self {
            thr, 
            sender: sender_com,
            receiver: receiver_com
        }
    }

    pub fn send_msg(&self, msg : Msg) {
        // Clean recv buffer
        loop {
            if self.receiver.try_recv().is_err() {
                break;
            }
        }

        self.sender.send(Some(msg)).expect("Thread crashed")
    }

    pub fn await_inactive(&self) -> Res {
        self.receiver.recv().expect("Recv failed")  // TODO: Improve error handling
    }

    pub fn kill(&self) -> &JoinHandle<()> {
        if !self.thr.is_finished() {
            self.sender.send(None).unwrap_or(());
        }

        &self.thr
    }
}

impl<Msg: Send + 'static, Res: Send + 'static> Drop for AsyncComms<Msg, Res> {
    fn drop(&mut self) {
        self.kill();
    }
}