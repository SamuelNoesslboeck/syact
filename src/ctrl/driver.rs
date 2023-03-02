use core::time::Duration;
use std::sync::Arc;
use std::thread;

use gpio::{GpioIn, GpioOut};
use gpio::sysfs::SysFsGpioOutput;

use crate::data::StepperVar;
use crate::{StepperConst, LinkedData, Gamma, Time, Omega, Delta, Alpha, Inertia, Force};
use crate::ctrl::types::*;
use crate::math;

/// ### Driver
/// Driver class for basic stepper motor operations
#[derive(Debug)]
pub struct StepperDriver 
{
    /// Stepper data
    pub consts : StepperConst,
    pub vars : StepperVar,

    /// The current direction of the driver, the bool value is written to the `pin_dir` GPIO pin\
    /// DO NOT WRITE TO THIS VALUE! Use the `Driver::set_dir()` function instead
    pub dir : bool,
    /// The current absolute position since set to a value
    pub pos : i64,

    lk : Arc<LinkedData>,

    /// Pin for defining the direction
    sys_dir : RaspPin,
    /// Pin for PWM Step pulses
    sys_step : RaspPin,
    /// Measurement pin
    sys_meas : RaspPin,

    /// Limit for minimum angle/step count
    limit_min : Option<Gamma>,
    /// Limit for maximum angle/step count
    limit_max : Option<Gamma>
}

impl StepperDriver {
    // Init
    /// Creates a new Driver from the given stepper `StepperData` \
    /// Pin numbers are based on the BCM-Scheme, not by absolute board numbers
    pub fn new(data : StepperConst, pin_dir : u16, pin_step : u16) -> Self {
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
            consts: data, 
            vars: StepperVar::ZERO, 

            dir: true, 
            pos: 0,

            lk: Arc::new(LinkedData::EMPTY),
            
            sys_dir,
            sys_step,
            sys_meas: RaspPin::ErrPin,

            limit_min: None,
            limit_max: None
        };

        // Write initial direction to output pins
        driver.set_dir(driver.dir);

        driver
    }

    pub fn new_save(data : StepperConst, pin_dir : u16, pin_step : u16) -> Result<Self, std::io::Error> {
        let mut driver = StepperDriver {
            consts: data, 
            vars: StepperVar::ZERO,

            dir: true, 
            pos: 0,

            lk: Arc::new(LinkedData::EMPTY),
            
            sys_dir: RaspPin::Output(SysFsGpioOutput::open(pin_dir.clone())?),
            sys_step: RaspPin::Output(SysFsGpioOutput::open(pin_step.clone())?),
            sys_meas: RaspPin::ErrPin,

            limit_min: None,
            limit_max: None
        };

        // Write initial direction to output pins
        driver.set_dir(driver.dir);

        Ok(driver)
    }

    #[inline]
    pub fn link(&mut self, lk : Arc<LinkedData>) {
        self.lk = lk;
    }  

    #[inline]
    pub fn set_meas(&mut self, sys_meas : RaspPin) {
        self.sys_meas = sys_meas;
    }

    // Misc
        /// Helper function for measurements with a single pin
        #[inline]
        pub fn __meas_helper(pin : &mut RaspPin) -> bool {
            match pin {
                RaspPin::Input(gpio_pin ) => {
                    gpio_pin.read_value().unwrap() == gpio::GpioValue::High
                },
                _ => true
            }
        }
    //

    // Movements
        /// Move a single step into the previously set direction. Uses `thread::sleep()` for step times, so the function takes `time` in seconds to process
        pub fn step(&mut self, time : Time, ufunc : &UpdateFunc) -> StepResult {
            match &mut self.sys_step {
                RaspPin::Output(pin) => {
                    let step_time_half : Duration = (time / 2.0).into();

                    pin.set_high().unwrap();
                    thread::sleep(step_time_half);
                    pin.set_low().unwrap();
                    thread::sleep(step_time_half);
                    
            
                    self.pos += if self.dir { 1 } else { -1 };
        
                    match self.limit_max {
                        Some(pos) => {
                            if self.get_gamma() > pos {
                                return StepResult::Break;
                            }
                        }, 
                        _ => { }
                    };
        
                    match self.limit_min {
                        Some(pos) => {
                            if self.get_gamma() < pos {
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

        pub fn accelerate(&mut self, stepcount : u64, omega : Omega, ufunc : &UpdateFunc) -> (StepResult, Vec<Time>) {
            let t_start = self.lk.s_f / math::start_frequency(&self.consts, &self.vars);
            let t_min = self.consts.step_time(omega);

            let mut o_last = Omega::ZERO;
            let mut t_total = t_start;
            let mut time_step = t_start;          // Time per step
            let mut i: u64 = 1;                         // Step count
            let mut curve = vec![];

            self.step(time_step, ufunc);
            curve.push(time_step);

            loop {
                if i >= stepcount {
                    break;
                }

                o_last = math::angluar_velocity_dyn(&self.consts, &self.vars, t_total, o_last, self.lk.u);
                time_step = self.consts.step_ang() / o_last * self.lk.s_f;
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

        pub fn drive_curve(&mut self, curve : &Vec<Time>) {
            for i in 0 .. curve.len() {
                self.step(curve[i], &UpdateFunc::None);
            }
        }

        pub fn steps(&mut self, stepcount : u64, omega : Omega, ufunc : UpdateFunc) -> StepResult {
            let ( result, mut curve ) = self.accelerate( stepcount / 2, omega, &ufunc);
            let time_step = self.consts.step_time(omega);
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

        pub fn drive(&mut self, dist : Delta, omega : Omega, ufunc : UpdateFunc) -> Delta {
            if !dist.is_normal() {
                return Delta::ZERO;
            } else if dist > Delta::ZERO {
                self.set_dir(true);
            } else if dist < Delta::ZERO {
                self.set_dir(false);
            }

            let steps : u64 = self.consts.ang_to_steps_dir(dist.into()).abs() as u64;
            self.steps(steps, omega, ufunc);

            Delta((steps as f32) * self.consts.step_ang())
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
        #[inline]
        pub fn get_gamma(&self) -> Gamma {
            Gamma(self.steps_to_ang_dir(self.pos))
        }   

        #[inline]
        pub fn write_gamma(&mut self, pos : Gamma) {
            self.pos = self.ang_to_steps_dir(pos.into());
        }
    //

    // Limits
        #[inline]
        pub fn set_limit(&mut self, min : Option<Gamma>, max : Option<Gamma>) {
            if min.is_some() {
                self.limit_min = min;
            }
            if max.is_some() {
                self.limit_max = max;
            }
        }

        pub fn get_limit_dest(&self, gamma : Gamma) -> Delta {
            return match self.limit_min {
                Some(ang) => {
                    if gamma < ang {
                        gamma - ang
                    } else { Delta::ZERO }
                },
                None => match self.limit_max {
                    Some(ang) => {
                        if gamma > ang {
                            gamma - ang
                        } else { Delta::ZERO }
                    },
                    None => Delta::NAN
                }
            };
        }

        pub fn set_endpoint(&mut self, set_gamma : Gamma) -> bool {
            if Self::__meas_helper(&mut self.sys_meas) {
                self.pos = self.ang_to_steps_dir(set_gamma.0);
    
                self.set_limit(
                    if self.dir { self.limit_min } else { Some(set_gamma) },
                    if self.dir { Some(set_gamma) } else { self.limit_max }
                );

                true
            } else {
                false
            }
        }
    //

    // Conversions
        #[inline]
        pub fn ang_to_steps(&self, ang : f32) -> u64 {
            (ang.abs() / self.consts.step_ang()).round() as u64
        }

        #[inline]
        pub fn ang_to_steps_dir(&self, ang : f32) -> i64 {
            (ang / self.consts.step_ang()).round() as i64
        }   

        #[inline]
        pub fn steps_to_ang(&self, steps : u64) -> f32 {
            steps as f32 * self.consts.step_ang()
        }

        #[inline]
        pub fn steps_to_ang_dir(&self, steps : i64) -> f32 {
            steps as f32 * self.consts.step_ang()
        }
    //

    // Loads
        #[inline]
        pub fn accel_dyn(&self, omega : Omega) -> Alpha {
            self.consts.alpha_max_dyn(math::torque_dyn(&self.consts, omega, self.lk.u), &self.vars)
        }

        #[inline]
        pub fn apply_load_inertia(&mut self, j : Inertia) {
            self.vars.j_load = j;
        }

        #[inline]
        pub fn apply_load_force(&mut self, t : Force) {
            self.vars.t_load = t;
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
