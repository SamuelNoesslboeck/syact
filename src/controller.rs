use std::{thread, time, vec};
use std::f32::consts::PI;

use gpio::{GpioOut, sysfs::*};

use crate::data::StepperData;
use crate::math::start_frequency;

// type UpdateLoadFunc = fn (&StepperData);
// type UpdatePosFunc = fn (&dyn StepperCtrl);

const PIN_ERR : u16 = 0xFF;

#[derive(Debug)]
pub enum RaspPin {
    ErrPin(),
    Output(SysFsGpioOutput),
    Input(SysFsGpioInput)
}

#[derive(Debug)]
pub enum LimitType {
    None,
    Angle(f32)
}

#[derive(Debug)]
pub enum LimitDest {
    NoLimitSet,
    NotReached,
    Minimum(f32),
    Maximum(f32)
}

/// A trait for structs used to control stepper motors through different methods
pub trait StepperCtrl
{
    // Data
    /// Get the data of the stepper that is being controlled
    fn get_data(&self) -> &StepperData;
    /// Returns a mutable reference to the stepperdata
    fn get_data_mut(&mut self) -> &mut StepperData;

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
    fn get_dir(&self) -> bool;
    /// Set the direction of the motor  \
    /// Directions are defined by the electronics and the user
    fn set_dir(&mut self, dir : bool);

    /// Returns the absolute position in radians
    fn get_abs_pos(&self) -> f32;    
    /// Returns the relative position (current rotation) in radians
    fn get_rel_pos(&self) -> f32;
    /// Overwrite absolute position
    fn write_pos(&mut self, pos : f32);

    /// Set the limit for the stepper motor
    fn set_limit(&mut self, min : LimitType, max : LimitType);
    /// Check if the stepper motor is in a limited position
    fn get_limit_dest(&self) -> LimitDest;

    /// Moves the motor in the current direction till the measure pin is true
    fn measure(&mut self, max_steps : u64, omega : f32) -> Option<()>;
    /// Limit distance
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
    // sys_mes : Option<SysFsGpioInput>,

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
            Err(_) => RaspPin::ErrPin()
        };

        let sys_step = match SysFsGpioOutput::open(pin_step.clone()) {
            Ok(val) => RaspPin::Output(val),
            Err(_) => RaspPin::ErrPin()
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

            limit_min: LimitType::None,
            limit_max: LimitType::None,
            // sys_mes: None,

            omega: 0.0
        };

        ctrl.set_dir(ctrl.dir);

        return ctrl;
    }
}

impl StepperCtrl for PwmStepperCtrl
{
    fn get_data(&self) -> &StepperData {
        return &self.data;    
    }

    fn get_data_mut(&mut self) -> &mut StepperData {
        return &mut self.data;
    }

    fn step(&mut self, time : f32) {
        let step_time_half = time::Duration::from_secs_f32(time / 2.0);
        
        match &mut self.sys_step {
            RaspPin::Output(pin) => {
                pin.set_high().unwrap();
                thread::sleep(step_time_half);
                pin.set_low().unwrap();
                thread::sleep(step_time_half);
                
        
                self.pos += if self.dir { 1 } else { -1 };
            },
            _ => { }
        };
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
            self.set_dir(false);
        }

        let steps : u64 = (distance.abs() / self.data.ang_dis()).round() as u64;
        self.steps(steps, omega);
        return steps as f32 * self.data.ang_dis();
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
            return 2.0 * PI * self.pos as f32 / self.data.n_s as f32;
        }

        fn get_rel_pos(&self) -> f32 {
            return 2.0 * PI * (self.pos % self.data.n_s as i64) as f32 / self.data.n_s as f32;
        }

        fn write_pos(&mut self, pos : f32) {
            self.pos = (self.data.n_s as f32 * pos / 2.0 / PI) as i64;
        }
    //

    // Limits
        fn set_limit(&mut self, min : LimitType, max : LimitType) {
            self.limit_min = min;
            self.limit_max = max;
        }
    
        fn get_limit_dest(&self) -> LimitDest {
            let pos = self.get_abs_pos();

            let match_b = |pdest : LimitDest| {
                match self.limit_max {
                    LimitType::Angle(ang) => {
                        if pos > ang {
                            return LimitDest::Minimum(pos - ang);
                        }

                        LimitDest::NotReached
                    },
                    _ => pdest
                }
            };

            match match self.limit_min {
                LimitType::Angle(ang) => {
                    if pos < ang {
                        return LimitDest::Minimum(pos - ang)
                    }

                    LimitDest::NotReached
                },
                _ => LimitDest::NoLimitSet
            } {
                LimitDest::NotReached => match_b(LimitDest::NotReached),
                LimitDest::NoLimitSet => match_b(LimitDest::NoLimitSet),
                other => other
            }
        }
    // 

    fn measure(&mut self, max_steps : u64, omega : f32) -> Option<()> {
        let mut curve = self.accelerate(max_steps / 2, omega);
        
        curve.reverse();
        self.drive_curve(&curve);

        return None;
    }

    fn debug_pins(&self) {
        dbg!(
            &self.sys_dir,
            &self.sys_step
        );
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
        /// Angle for extent
        pub fn phi_c(&self, dis_c : f32) -> f32 {
            dis_c / self.rte_ratio
        }

        /// Extent for angle
        pub fn dis_c(&self, phi_c : f32) -> f32 {
            phi_c * self.rte_ratio
        }

        /// Angular speed for linear velocity
        pub fn omega_c(&self, v_c : f32) -> f32 {
            v_c / self.rte_ratio
        }

        /// Linear velocity for angular speed
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

    pub fn write_length(&mut self, dis : f32) {
        self.ctrl.write_pos(self.phi_c(dis));
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
    pub fn new(cylinder : Cylinder, l_a : f32, l_b : f32) -> Self
    {
        let mut tri = CylinderTriangle {
            l_a, 
            l_b,
            cylinder 
        };

        tri.cylinder.write_length(l_a.max(l_b));

        return tri;
    }

    pub fn length_for_gamma(&self, gam : f32) -> f32 {
        (self.l_a.powi(2) + self.l_b.powi(2) + 2.0 * self.l_a * self.l_b * gam.cos()).powf(0.5)
    }

    pub fn get_gamma(&self) -> f32 {
        ((self.cylinder.length().powi(2) as f32 - self.l_a.powi(2) - self.l_b.powi(2)) / 2.0 / self.l_a / self.l_b).acos()
    }

    pub fn set_gamma(&mut self, gam : f32, v_max : f32) {
        self.cylinder.extend(self.length_for_gamma(gam), v_max);
    }

    // pub fn write_gamma(&mut self, gam : f32) {
        
    // }
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