use std::{thread, time, vec};

use gpio::{GpioIn, GpioOut, sysfs::*};

use crate::data::StepperData;
use crate::math::{start_frequency, angluar_velocity_dyn};

// type UpdateLoadFunc = fn (&StepperData);
// type UpdatePosFunc = fn (&dyn StepperCtrl);

/// Constant for expressing an incorrect pin number
const PIN_ERR : u16 = 0xFF;

/// Pin helper enum for safe use in Debug enviroments
#[derive(Debug)]
pub enum RaspPin {
    ErrPin,
    Output(SysFsGpioOutput),
    Input(SysFsGpioInput)
}

/// Different types of enums and their values
#[derive(Debug)]
pub enum LimitType {
    None,
    Angle(i64),
    Distance(f32)
}

/// Current status of the limits set
#[derive(Debug)]
pub enum LimitDest {
    NoLimitSet,
    NotReached,
    Minimum(f32),
    Maximum(f32)
}

impl LimitDest {
    pub fn reached(&self) -> bool {
        match self {
            LimitDest::Maximum(_) => true,
            LimitDest::Minimum(_) => true,
            _ => false
        }
    }
}

/// Update functions for updating stepper data in 
pub enum UpdateFunc {
    None,
    Data(fn (StepperData) -> StepperData, u64),
    Break(for<'a> fn (&'a mut RaspPin) -> bool, u64)
}

/// Result of a stepper operation
pub enum StepResult {
    None,
    Break,
    Error
}

/// A trait for structs used to control stepper motors through different methods
pub trait StepperCtrl
{
    // Init
    /// Initializes measuring systems
    fn init_meas(&mut self, pin_meas : u16); 

    // Data
    /// Get the data of the stepper that is being controlled
    fn get_data(&self) -> &StepperData;
    /// Returns a mutable reference to the stepperdata
    fn get_data_mut(&mut self) -> &mut StepperData;

    // Conversions
    /// Converts the given angle into the amount of steps required to come as close as possible to that distance
    fn ang_to_steps(&self, ang : f32) -> u64;
    /// Converts the given angle into the amount of steps required to come as close as possible to that distance, if the angle is negative, the amount of steps in negative 
    fn ang_to_steps_dir(&self, ang : f32) -> i64;
    /// Converts the given steps into an angle
    fn steps_to_ang(&self, steps : u64) -> f32;
    /// Converts the given steps into an angle, including directions
    fn steps_to_ang_dir(&self, steps : i64) -> f32;

    /// Move a single step
    fn step(&mut self, time : f32, ufunc : &UpdateFunc) -> StepResult;

    /// Accelerates the motor as fast as possible to the given speed, canceling if it takes any longer than the stepcount given. The function returns the steps needed for the acceleration process
    fn accelerate(&mut self, stepcount : u64, omega : f32, ufunc : &UpdateFunc) -> (StepResult, Vec<f32>);
    /// Drive a curve of step times, represented by a list
    fn drive_curve(&mut self, curve : &Vec<f32>);
    /// Move a number of steps as fast as possible, the steps will be traveled without 
    fn steps(&mut self, stepcount : u64, omega : f32, ufunc : UpdateFunc) -> StepResult;
    /// Drive a certain distance, returns the actual distance traveled
    fn drive(&mut self, distance : f32, omega : f32, ufunc : UpdateFunc) -> f32;
    /// Move a number of steps safefy (with loads included)
        // fn steps_save(&mut self, stepcount : u64, omega : f32, up_load : UpdateLoadFunc);
        // fn steps_update(&mut self, stepcount : u64, omega : f32, up_load : UpdateLoadFunc, up_pos : UpdatePosFunc);
    // Stops the motor as fast as possible
    fn stop(&mut self) -> u64;

    /// Get the current speed
    fn get_speed(&self) -> f32;
    /// Sets the speed after accelerating
    fn set_speed(&mut self, omega : f32);

    /// Get the current direction of the motor
    fn get_dir(&self) -> bool;
    /// Set the direction of the motor  \
    /// Directions are defined by the electronics and the user
    fn set_dir(&mut self, dir : bool);

    /// Returns the absolute position in radians
    fn get_abs_pos(&self) -> f32;    
    /// Overwrite absolute position
    fn write_pos(&mut self, pos : f32);

    /// Set the limit for the stepper motor
    fn set_limit(&mut self, min : LimitType, max : LimitType);
    /// Check if the stepper motor is in a limited position
    fn get_limit_dest(&self, pos : i64) -> LimitDest;

    /// Moves the motor in the current direction till the measure pin is true
    fn measure(&mut self, max_steps : u64, omega : f32, dir : bool, set_pos : i64, accuracy : u64);
    /// Returns the measurement pin
    fn get_meas(&mut self) -> &mut RaspPin;

    // Loads
    /// Applies a load torque to the stepper motor that will affect calculations
    fn apply_load_t(&mut self, t : f32);
    /// Applies a load inertia to the stepper motor that will affect calculations
    fn apply_load_j(&mut self, j : f32);
    // 

    /// Display all pins
    fn debug_pins(&self);
}

pub struct PwmStepperCtrl
{
    /// Stepper data
    pub data : StepperData,
    /// Safety factor for load and speed calculations
    pub sf : f32,
    /// Curve factor for slower acceleration
    pub cf : f32, 

    /// Pin for controlling direction
    pub pin_dir : u16,
    /// Pin for controlling steps
    pub pin_step : u16,
    /// Pin for messuring distances
    pub pin_mes : u16, 

    /// The current direction (true for right, false for left)
    dir : bool,
    /// The current absolute position since set to a value
    pos : i64,

    /// Pin for defining the direction
    sys_dir : RaspPin,
    /// Pin for PWM Step pulses
    sys_step : RaspPin,
    sys_meas : RaspPin,

    limit_min : LimitType,
    limit_max : LimitType,

    omega : f32
}

impl PwmStepperCtrl
{   
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

        let mut ctrl = PwmStepperCtrl { 
            data: data,
            sf: 1.5, 
            cf: 0.9,
            pin_dir: pin_dir, 
            pin_step: pin_step, 
            pin_mes: PIN_ERR, 
            dir: true, 
            pos: 0,
            
            sys_dir,
            sys_step,
            sys_meas: RaspPin::ErrPin,

            limit_min: LimitType::None,
            limit_max: LimitType::None,

            omega: 0.0
        };

        ctrl.set_dir(ctrl.dir);

        return ctrl;
    }

    fn __meas_helper(pin : &mut RaspPin) -> bool {
        match pin {
            RaspPin::Input(gpio_pin) => {
                gpio_pin.read_value().unwrap() == gpio::GpioValue::High
            },
            _ => false
        }
    }
}

impl StepperCtrl for PwmStepperCtrl
{
    // Init
        fn init_meas(&mut self, pin_mes : u16) {
            self.pin_mes = pin_mes;
            self.sys_meas = match SysFsGpioInput::open(pin_mes) {
                Ok(val) => RaspPin::Input(val),
                Err(_) => RaspPin::ErrPin
            }
        }
    //

    fn get_data(&self) -> &StepperData {
        return &self.data;    
    }

    fn get_data_mut(&mut self) -> &mut StepperData {
        return &mut self.data;
    }

    // Conversions
        fn ang_to_steps(&self, ang : f32) -> u64 {
            (ang.abs() / self.data.step_ang()).round() as u64
        }

        fn ang_to_steps_dir(&self, ang : f32) -> i64 {
            (ang / self.data.step_ang()).round() as i64
        }

        fn steps_to_ang(&self, steps : u64) -> f32 {
            steps as f32 * self.data.step_ang()
        }

        fn steps_to_ang_dir(&self, steps : i64) -> f32 {
            steps as f32 * self.data.step_ang()
        }
    //

    fn step(&mut self, time : f32, ufunc : &UpdateFunc) -> StepResult {
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
                        if self.pos > pos {
                            return StepResult::Break;
                        }
                    }, 
                    _ => { }
                };

                match self.limit_min {
                    LimitType::Angle(pos) => {
                        if self.pos < pos {
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

    fn accelerate(&mut self, stepcount : u64, omega : f32, ufunc : &UpdateFunc) -> (StepResult, Vec<f32>) {
        let t_start = self.sf / start_frequency(&self.data);
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
                self.set_speed(omega);
                break;
            }

            o_last = angluar_velocity_dyn(&self.data, t_total, o_last);
            time_step = self.data.step_ang() / o_last * self.sf;
            t_total += time_step;

            if time_step < t_min {
                self.set_speed(omega);
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

    fn drive_curve(&mut self, curve : &Vec<f32>) {
        for i in 0 .. curve.len() {
            self.step(curve[i], &UpdateFunc::None);
        }
    }

    fn steps(&mut self, stepcount : u64, omega : f32, ufunc : UpdateFunc) -> StepResult {
        let ( result, mut curve ) = self.accelerate(stepcount / 2, omega, &ufunc);
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
        
        self.set_speed(0.0);

        StepResult::None
    }

    fn drive(&mut self, distance : f32, omega : f32, ufunc : UpdateFunc) -> f32 {
        if distance == 0.0 {
            return 0.0;
        } else if distance > 0.0 {
            self.set_dir(true);
        } else if distance < 0.0 {
            self.set_dir(false);
        }

        let steps : u64 =  self.ang_to_steps(distance);
        self.steps(steps, omega, ufunc);
        return steps as f32 * self.data.step_ang();
    }
    
    fn stop(&mut self) -> u64 {
        0 // TODO
    }

    // Speed
        fn get_speed(&self) -> f32 {
            return self.omega;
        }

        fn set_speed(&mut self, omega : f32) {
            self.omega = omega;
        }
    //

    // Direction
        fn get_dir(&self) -> bool {
            return self.dir;
        }
        
        fn set_dir(&mut self, dir : bool) {
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
    //

    // Position
        fn get_abs_pos(&self) -> f32 {
            self.steps_to_ang_dir(self.pos)
        }

        fn write_pos(&mut self, pos : f32) {
            self.pos = self.ang_to_steps_dir(pos);
        }
    //

    // Limits
        fn set_limit(&mut self, min : LimitType, max : LimitType) {
            self.limit_min = min;
            self.limit_max = max;
        }
    
        fn get_limit_dest(&self, pos : i64) -> LimitDest {
            let res_min = match self.limit_min {
                LimitType::Angle(ang) => {
                    if pos < ang {
                        return LimitDest::Minimum((pos - ang) as f32 * self.data.step_ang())
                    }

                    LimitDest::NotReached
                },
                _ => LimitDest::NoLimitSet
            };
             
            if !res_min.reached() {
                return match self.limit_max {
                    LimitType::Angle(ang) => {
                        if pos > ang {
                            return LimitDest::Maximum((pos - ang) as f32 * self.data.step_ang());
                        }

                        LimitDest::NotReached
                    },
                    _ => res_min
                };
            } else {
                return res_min;
            }
        }
    // 

    fn measure(&mut self, max_steps : u64, omega : f32, dir : bool, set_pos : i64, accuracy : u64) {
        // TODO: Add failsafes

        self.set_dir(dir);
        self.steps(max_steps, omega, UpdateFunc::Break(Self::__meas_helper, accuracy));
        
        // Successful measurement
        if Self::__meas_helper(&mut self.sys_meas) {
            self.pos = set_pos;

            self.set_limit(
                if dir { LimitType::None } else { LimitType::Angle(set_pos) },
                if dir { LimitType::Angle(set_pos) } else { LimitType::None }
            )
        }
    }

    fn get_meas(&mut self) -> &mut RaspPin {
        &mut self.sys_meas
    }

    // Loads
        fn apply_load_j(&mut self, j : f32) {
            self.data.j_load = j;
        }

        fn apply_load_t(&mut self, t : f32) {
            self.data.t_load = t;
        }
    // 

    fn debug_pins(&self) {
        dbg!(
            &self.sys_dir,
            &self.sys_step,
            &self.sys_meas
        );
    }
}