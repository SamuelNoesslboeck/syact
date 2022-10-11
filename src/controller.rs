use std::time::Duration;
use std::{thread, time, vec};
use std::f64::consts::PI;
use std::ops::Index;

use gpio::{sysfs::*, GpioOut};

use crate::data::StepperData;
use crate::math::{angluar_velocity, start_frequency};

type UpdateLoadFunc = fn (&StepperData);
type UpdatePosFunc = fn (&dyn StepperCtrl);

const PIN_ERR : u16 = 0xFF;

pub trait StepperCtrl
{
    /// Move a single step
    fn step(&mut self);

    /// Move a number of steps as fast as possible, the steps will be traveled without 
    fn steps(&mut self, stepcount : u64, omega : f64);
    /// Move a number of steps safefy (with loads included)
    fn steps_save(&mut self, stepcount : u64, omega : f64, up_load : UpdateLoadFunc);
    fn steps_update(&mut self, stepcount : u64, omega : f64, up_load : UpdateLoadFunc, up_pos : UpdatePosFunc);
    // Stops the motor as fast as possible
    fn stop(&self);
    /// Let's the motor accelerate to the given speed as fast as possible
    fn set_speed(&self, omega : f64);

    /// Get the current direction of the motor
    /// (True for right, False for left)
    fn get_dir(&self) -> bool;
    /// Set the direction of the motor
    /// (True for right, False for left)
    fn set_dir(&mut self, dir : bool);

    /// Returns the absolute position in radians
    fn get_abs_pos(&self) -> f64;    
    /// Returns the relative position (current rotation) in radians
    fn get_rel_pos(&self) -> f64;

    // /// Sets the current absolute position
    // fn set_absolute_pos(&mut self) -> f64;
    // /// Sets the current relative position
    // fn set_relative_pos(&mut self) -> f64;
}

pub struct PwmStepperCtrl
{
    /// Stepper data
    pub data : StepperData,
    /// Safety factor for load and speed calculations
    pub sf : f64,

    /// Pin for controlling direction
    pub pin_dir : u16,
    /// Pin for controlling steps
    pub pin_step : u16,
    /// Pin for messuring distances
    pub pin_mes : u16, 

    /// The current direction (true for right, false for left)
    pub dir : bool,
    /// The current absolute position since set to a value
    pub pos : i64,

    /// Time span of holding the high signal when processing a step
    pub t_stephold_high : time::Duration,

    sys_dir : SysFsGpioOutput,
    sys_step : SysFsGpioOutput,
    sys_mes : Option<SysFsGpioInput>
}

impl PwmStepperCtrl
{   
    pub fn new(data : StepperData, pin_dir : u16, pin_step : u16) -> Self {
        return PwmStepperCtrl { 
            data: data,
            sf: 1.0, 
            pin_dir: pin_dir, 
            pin_step: pin_step, 
            pin_mes: PIN_ERR, 
            dir: true, 
            pos: 0,
            t_stephold_high: time::Duration::from_micros(100),
            
            sys_dir: SysFsGpioOutput::open(pin_dir).unwrap(),
            sys_step: SysFsGpioOutput::open(pin_step).unwrap(),
            sys_mes: None
        };
    }
}

impl StepperCtrl for PwmStepperCtrl
{
    fn step(&mut self) {
        self.sys_step.set_high().unwrap();
        thread::sleep(self.t_stephold_high);
        self.sys_step.set_low().unwrap();

        self.pos += if self.dir { 1 } else { -1 };
    }

    fn steps(&mut self, stepcount : u64, omega : f64) {
        let mut curve : Vec<f64> = vec![
            1.0 / start_frequency(&self.data)
        ];

        self.step();

        let t_min = 2.0 * PI / self.data.n_s as f64 / omega;
        let mut t_total = curve[0];
        for i in 0 .. stepcount / 2 - 1 {
            thread::sleep(Duration::from_secs_f64(*curve.index(i as usize)));
            self.step();
            curve.push(2.0 * PI / (self.data.n_s as f64) / angluar_velocity(&self.data, t_total));

            if *curve.index(i as usize) > t_min {
                t_total += *curve.index(i as usize + 1);
            } else {
                *curve.last_mut().unwrap() = t_min;
            }
        }

        if (stepcount % 2) == 1 {
            thread::sleep(Duration::from_secs_f64(*curve.last().unwrap()));
            self.step()
        }

        for i in 1 .. stepcount / 2 + 1 {
            thread::sleep(Duration::from_secs_f64(*curve.index((stepcount/2 - i) as usize)));
            self.step();
        }
    }

    fn steps_save(&mut self, stepcount : u64, omega : f64, up_load : UpdateLoadFunc) {
        
    }
    
    fn stop(&self) {
        
    }

    fn set_speed(&self, omega : f64) {
        
    }

    fn get_dir(&self) -> bool {
        return self.dir;
    }
    
    fn set_dir(&mut self, dir : bool) {
        self.dir = dir;
    }

    fn get_abs_pos(&self) -> f64 {
        return 2.0 * PI * self.pos as f64 / self.data.n_s as f64;
    }

    fn get_rel_pos(&self) -> f64 {
        return 2.0 * PI * (self.pos % self.data.n_s as i64) as f64 / self.data.n_s as f64;
    }
}