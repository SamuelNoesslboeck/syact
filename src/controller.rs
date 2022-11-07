use std::{thread, time, vec};
use std::f32::consts::PI;

use gpio::{sysfs::*, GpioOut};

use crate::data::StepperData;
use crate::math::start_frequency;

// type UpdateLoadFunc = fn (&StepperData);
// type UpdatePosFunc = fn (&dyn StepperCtrl);

const PIN_ERR : u16 = 0xFF;

/// A trait for structs used to control stepper motors through different methods
pub trait StepperCtrl
{
    // Data
    /// Get the data of the stepper that is being controlled
    fn get_data(&self) -> &StepperData;

    /// Move a single step
    fn step(&mut self, time : f32);

    /// Accelerates the motor as fast as possible to the given speed, canceling if it takes any longer than the stepcount given. The function returns the steps needed for the acceleration process
    fn accelerate(&mut self, stepcount : u64, omega : f32) -> Vec<f32>;
    /// Drive a curve of step times, represented by a list
    fn drive_curve(&mut self, curve : &Vec<f32>);
    /// Move a number of steps as fast as possible, the steps will be traveled without 
    fn steps(&mut self, stepcount : u64, omega : f32);
    /// Drive a certain distance, returns the actual distance traveled
    fn drive(&mut self, distance : f32, omega : f32) -> f32;
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
    /// (True for right, False for left)
    fn get_dir(&self) -> bool;
    /// Set the direction of the motor
    /// (True for right, False for left)
    fn set_dir(&mut self, dir : bool);

    /// Returns the absolute position in radians
    fn get_abs_pos(&self) -> f32;    
    /// Returns the relative position (current rotation) in radians
    fn get_rel_pos(&self) -> f32;

    /// Moves the motor in the current direction till the measure pin is true
    fn measure(&mut self, max_steps : u64, omega : f32) -> Option<()>;

    // /// Sets the current absolute position
    // fn set_absolute_pos(&mut self) -> f32;
    // /// Sets the current relative position
    // fn set_relative_pos(&mut self) -> f32;
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
    sys_dir : SysFsGpioOutput,
    /// Pin for PWM Step pulses
    sys_step : SysFsGpioOutput,
    // sys_mes : Option<SysFsGpioInput>,

    omega : f32
}

impl PwmStepperCtrl
{   
    pub fn new(data : StepperData, pin_dir : u16, pin_step : u16) -> Self {
        return PwmStepperCtrl { 
            data: data,
            sf: 1.5, 
            cf: 0.9,
            pin_dir: pin_dir, 
            pin_step: pin_step, 
            pin_mes: PIN_ERR, 
            dir: true, 
            pos: 0,
            
            sys_dir: SysFsGpioOutput::open(pin_dir).unwrap(),
            sys_step: SysFsGpioOutput::open(pin_step).unwrap(),
            // sys_mes: None,

            omega: 0.0
        };
    }
}

impl StepperCtrl for PwmStepperCtrl
{
    fn get_data(&self) -> &StepperData {
        return &self.data;    
    }

    fn step(&mut self, time : f32) {
        let step_time_half = time::Duration::from_secs_f32(time / 2.0);

        self.sys_step.set_high().unwrap();
        thread::sleep(step_time_half);
        self.sys_step.set_low().unwrap();
        thread::sleep(step_time_half);

        self.pos += if self.dir { 1 } else { -1 };
    }

    fn accelerate(&mut self, stepcount : u64, omega : f32) -> Vec<f32> {
        let t_start = self.sf / start_frequency(&self.data);
        let t_min = self.data.time_step(omega);

        let mut time_step : f32;      // Time per step
        let mut i: u64 = 1;                 // Step count
        let mut curve = vec![];

        loop {
            if i > stepcount {
                self.set_speed(omega);
                break;
            }

            time_step = t_start / (i as f32).powf(0.5 * self.cf);

            if time_step < t_min {
                self.set_speed(omega);
                break;
            }

            self.step(time_step);
            curve.push(time_step);
            i += 1;
        }

        return curve;
    }

    fn drive_curve(&mut self, curve : &Vec<f32>) {
        for i in 0 .. curve.len() {
            self.step(curve[i]);
        }
    }

    fn steps(&mut self, stepcount : u64, omega : f32) {
        let curve = self.accelerate(stepcount / 2, omega);
        let time_step = self.data.time_step(omega);
        let last = curve.last().unwrap_or(&time_step);

        println!("{} / {} / {}", curve.len(), last, time_step);

        for _ in curve.len() .. (stepcount / 2) as usize {
            self.step(time_step);
        }

        if (stepcount % 2) == 1 {
            self.step(*last);
        }

        for _ in curve.len() .. (stepcount / 2) as usize {
            self.step(time_step);
        }

        self.drive_curve(&curve);
        self.set_speed(0.0);
    }

    fn drive(&mut self, distance : f32, omega : f32) -> f32 {
        if distance == 0.0 {
            return 0.0;
        } else if distance > 0.0 {
            self.set_dir(true);
        } else if distance < 0.0 {
            self.set_dir(true);
        }

        let steps : u64 = (distance.abs() / self.data.ang_dis()).round() as u64;
        self.steps(steps, omega);
        return steps as f32 * self.data.ang_dis();
    }
    
    fn stop(&mut self) -> u64 {
        0 // TODO
    }

    fn get_speed(&self) -> f32 {
        return self.omega;
    }

    fn set_speed(&mut self, omega : f32) {
        self.omega = omega;
    }

    fn get_dir(&self) -> bool {
        return self.dir;
    }
    
    fn set_dir(&mut self, dir : bool) {
        self.dir = dir;

        self.sys_dir.set_value(dir).unwrap();
    }

    fn get_abs_pos(&self) -> f32 {
        return 2.0 * PI * self.pos as f32 / self.data.n_s as f32;
    }

    fn get_rel_pos(&self) -> f32 {
        return 2.0 * PI * (self.pos % self.data.n_s as i64) as f32 / self.data.n_s as f32;
    }

    fn measure(&mut self, max_steps : u64, omega : f32) -> Option<()> {
        let mut curve = self.accelerate(max_steps / 2, omega);
        
        curve.reverse();
        self.drive_curve(&curve);

        return None;
    }
}

pub struct Cylinder
{
    /// Data of the connected stepper motor
    pub ctrl : Box<dyn StepperCtrl>,

    /// Distance traveled per rad [in mm]
    pub rte_ratio : f32,
    
    /// Minimal extent [in mm]
    pub pos_min : f32,
    /// Maximal extent [in mm]
    pub pos_max : f32
}

impl Cylinder
{
    /// Create a new cylinder instance
    pub fn new(ctrl : Box<dyn StepperCtrl>, rte_ratio : f32, pos_min : f32, pos_max : f32) -> Self {
        return Cylinder {
            ctrl,
            rte_ratio,
            pos_min,        // TODO: Use min and max
            pos_max
        };
    }

    /// Get the stepper motor data for the cylinder
    pub fn data(&self) -> &StepperData {
        self.ctrl.get_data()
    }

    // Conversions
        pub fn phi_c(&self, dis_c : f32) -> f32 {
            dis_c / self.rte_ratio
        }

        pub fn dis_c(&self, phi_c : f32) -> f32 {
            phi_c * self.rte_ratio
        }

        pub fn omega_c(&self, v_c : f32) -> f32 {
            v_c / self.rte_ratio
        }

        pub fn v_c(&self, omega : f32) -> f32 {
            omega * self.rte_ratio
        }
    //

    /// Extend the cylinder by a given distance _dis_ (in mm) with the maximum velocity _v max_ (in mm/s), returns the actual distance traveled
    pub fn extend(&mut self, dis : f32, v_max : f32) -> f32 {
        let steps = (self.phi_c(dis) / self.data().ang_dis()).floor() as u64;

        self.ctrl.steps(steps, self.omega_c(v_max));

        return steps as f32 * self.data().ang_dis();
    }

    /// Returns the extension of the cylinder
    pub fn length(&self) -> f32 {
        return self.dis_c(self.ctrl.get_abs_pos());
    }
}

pub struct CylinderTriangle 
{
    // Triangle
    pub l_a : f32,
    pub l_b : f32,

    // Cylinder length
    pub cylinder : Cylinder
}

impl CylinderTriangle 
{
    pub fn length_for_gamma(&self, gam : f32) -> f32 {
        (self.l_a.powi(2) + self.l_b.powi(2) + 2.0 * self.l_a * self.l_b * gam.cos()).powf(0.5)
    }

    pub fn gamma(&self) -> f32 {
        ((self.cylinder.length().powi(2) as f32 + self.l_a.powi(2) + self.l_b.powi(2)) / 2.0 / self.l_a / self.l_b).acos()
    }

    pub fn set_gamma(&mut self, gam : f32, v_max : f32) {
        self.cylinder.extend(self.length_for_gamma(gam), v_max);
    }
}

pub struct GearBearing 
{
    pub ctrl : Box<dyn StepperCtrl>,
    
    pub ratio : f32
}

impl GearBearing 
{
    pub fn get_pos(&self) -> f32 {
        self.ctrl.get_abs_pos() * self.ratio
    }

    pub fn set_pos(&mut self, pos : f32, omega : f32) -> f32 {
        self.ctrl.drive((pos - self.get_pos()) / self.ratio, omega)
    }
}