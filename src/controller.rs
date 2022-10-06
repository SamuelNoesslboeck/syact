use std::{thread, time};
use std::f64::consts::PI;

use gpio::GpioOut;
use gpio::sysfs::{SysFsGpioInput, SysFsGpioOutput};

use crate::data::StepperData;

const PIN_ERR : u16 = 0xFFFF;

pub trait StepperCtrl
{
    /// Move a single step
    fn step(&mut self);

    /// Move a number of steps as fast as possible, the steps will be traveled without 
    fn steps(&self, stepcount : u64);
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
    /// Min time of 
    pub t_stephold_low : time::Duration,

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
            t_stephold_high: time::Duration::from_millis(1),
            t_stephold_low: time::Duration::from_millis(1),

            sys_dir: SysFsGpioOutput::open(pin_dir).unwrap(),
            sys_step: SysFsGpioOutput::open(pin_step).unwrap(),
            sys_mes: None
        };
    }
}

impl StepperCtrl for PwmStepperCtrl
{
    fn step(&mut self) {
        self.sys_dir.set_high();
        thread::sleep(self.t_stephold_high);
        self.sys_dir.set_low();
        thread::sleep(self.t_stephold_low);

        self.pos += if self.dir { 1 } else { -1 };
    }

    fn steps(&self, stepcount : u64) {
        // Integrate acceleration curves
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