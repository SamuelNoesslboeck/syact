use super::*; 

/// ### Driver
/// Driver class for basic stepper motor operations
#[derive(Debug)]
pub struct StepperDriver 
{
    /// Stepper data
    pub data : StepperConst,

    /// The current direction of the driver, the bool value is written to the `pin_dir` GPIO pin\
    /// DO NOT WRITE TO THIS VALUE! Use the `Driver::set_dir()` function instead
    pub dir : bool,
    /// The current absolute position since set to a value
    pub pos : i64,

    pub j_load : f32,
    pub t_load : f32,

    lk : Arc<LinkedData>,

    /// Pin for defining the direction
    sys_dir : RaspPin,
    /// Pin for PWM Step pulses
    sys_step : RaspPin,
    /// Measurement pin
    sys_meas : RaspPin,

    /// Limit for minimum angle/step count
    limit_min : Option<f32>,
    /// Limit for maximum angle/step count
    limit_max : Option<f32>
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
            data, 
            dir: true, 
            pos: 0,

            j_load: 0.0,
            t_load: 0.0,

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
            data, 
            dir: true, 
            pos: 0,

            t_load: 0.0,
            j_load: 0.0,

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

    pub fn link(&mut self, lk : Arc<LinkedData>) {
        self.lk = lk;
    }

    pub fn set_meas(&mut self, sys_meas : RaspPin) {
        self.sys_meas = sys_meas;
    }

    // Misc
        /// Helper function for measurements with a single pin
        pub fn __meas_helper(pin : &mut RaspPin) -> bool {
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
                        Some(pos) => {
                            if self.get_dist() > pos {
                                return StepResult::Break;
                            }
                        }, 
                        _ => { }
                    };
        
                    match self.limit_min {
                        Some(pos) => {
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
            let t_start = self.lk.s_f / start_frequency(&self.data, self.t_load, self.j_load);
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

                o_last = angluar_velocity_dyn(&self.data, t_total, o_last, self.t_load, self.j_load, self.lk.u);
                time_step = self.data.step_ang() / o_last * self.lk.s_f;
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

        // pub fn accelerate(&mut self, stepcount : f32, )

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

        pub fn drive(&mut self, dist : f32, omega : f32, ufunc : UpdateFunc) -> f32 {
            if dist == 0.0 {
                return 0.0;
            } else if dist > 0.0 {
                self.set_dir(true);
            } else if dist < 0.0 {
                self.set_dir(false);
            }

            let steps : u64 = self.data.ang_to_steps_dir(dist).abs() as u64;
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

        // pub fn lin_move(&mut self, dist : f32, vel_0 : f32, vel_max : f32) -> f32 {
        // }
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
        pub fn set_limit(&mut self, min : Option<f32>, max : Option<f32>) {
            self.limit_min = min;
            self.limit_max = max;
        }

        pub fn get_limit_dest(&self, pos : f32) -> f32 {
            return match self.limit_min {
                Some(ang) => {
                    if pos < ang {
                        pos - ang
                    } else { 0.0 }
                },
                None => match self.limit_max {
                    Some(ang) => {
                        if pos > ang {
                            pos - ang
                        } else { 0.0 }
                    },
                    None => 0.0
                }
            };
        }

        pub fn set_endpoint(&mut self, set_pos : f32) -> bool {
            if Self::__meas_helper(&mut self.sys_meas) {
                self.pos = self.ang_to_steps_dir(set_pos);
    
                self.set_limit(
                    if self.dir { self.limit_min } else { Some(set_pos) },
                    if self.dir { Some(set_pos) } else { self.limit_max }
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
        pub fn accel_dyn(&self, omega : f32) -> f32 {
            self.data.alpha_max_dyn(torque_dyn(&self.data, omega, self.lk.u), self.t_load, self.j_load)
        }

        pub fn apply_load_inertia(&mut self, j : f32) {
            self.j_load = j;
        }

        pub fn apply_load_force(&mut self, t : f32) {
            self.t_load = t;
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
