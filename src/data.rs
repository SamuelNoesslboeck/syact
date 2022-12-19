//! ### Data subfile
//! 
//! Structs for storing stepper motor data and performing basic calculations \
//! NOTE: If any of the formulas are unclear, please have a look at \
//! the article on stepper motors included in this create under "article/article.pdf"

use serde::{Serialize, Deserialize};
use std::f32::consts::PI;

/// ### `StepperData`
/// A collection of the most relevant variables Unit stepper calculation 
/// ```
/// use stepper_lib::StepperData;
/// 
/// // Create the data from an standard motor
/// let mut data = StepperData::mot_17he15_1504s(
///     12.0,   // The motor is supplied with 12 Volts
///     1.5     // Safety factor of 1.5 to assure correct movements
/// );
/// 
/// data.apply_load_t(0.1);     // Apply a load torque of 0.1 Nm 
/// assert_eq!(data.t_s - data.t_load, data.t());
/// 
/// ``` 
/// Supports JSON-Serialize and Deserialize with the `serde_json` library
#[derive(Serialize, Deserialize)]
pub struct StepperData
{
    /// Motor voltage [Unit V]
    pub u : f32,
    /// Max phase current [Unit A]
    pub i_max : f32,
    /// Motor inductence [Unit H]
    pub l : f32,

    /// Step count per revolution [Unit (1)]
    pub n_s : u64,
    /// Coil pair count (n_s / 2) Unit many cases [Unit (1)]
    pub n_c : u64,
    /// Stall current [Unit Nm]
    pub t_s : f32,
    /// Inhertia moment [Unit kg*m^2]
    pub j_s : f32,

    // Dynamic
    /// Load torque [Unit Nm]
    pub t_load : f32,   
    /// Load inertia [Unit kg*m^2]
    pub j_load : f32,

    /// Safety factor [Unit (1)]
    pub sf : f32
}

impl StepperData
{
    /// ### Stepper motor 17HE15-1504S
    /// Values for standard stepper motor \
    /// Link: https://www.omc-stepperonline.com/de/e-serie-nema-17-bipolar-42ncm-59-49oz-in-1-5a-42x42x38mm-4-draehte-w-1m-kabel-verbinder-17he15-1504s
    pub fn mot_17he15_1504s(u : f32, sf : f32) -> Self {
        return StepperData { 
            u, 
            i_max: 1.5, 
            l: 0.004, 
            n_s: 200, 
            n_c: 100,
            t_s: 0.42, 
            j_s: 0.000_005_7,

            t_load: 0.0,
            j_load: 0.0,

            sf
        };
    }

    /// The maximum angular acceleration of the motor (in stall) [Unit s^-2]
    pub fn alpha_max(&self) -> f32 {
        self.t() / self.j()
    }

    /// The maximum angular acceleration of the motor, with a modified torque t_s [Unit s^-2]
    pub fn alpha_max_dyn(&self, t_s : f32) -> f32 {
        self.t_dyn(t_s) / self.j()
    }

    /// The inductivity constant [Unit s]
    pub fn tau(&self) -> f32 {
        self.i_max * self.l / self.u
    }

    /// Time per step for the given omega [Unit s]
    pub fn step_time(&self, omega : f32) -> f32 {
        2.0 * PI / (self.n_s as f32) / omega
    }

    /// Omega for time per step [Unit 1/s]
    pub fn omega(&self, step_time : f32) -> f32 {
        (self.n_s as f32) / 2.0 / PI / step_time
    }

    /// Get the angular distance of a step Unit rad [Unit 1]
    pub fn step_ang(&self) -> f32 {
        2.0 * PI / self.n_s as f32
    }

    // Load calculations
        /// Max motor torque when having a load [Unit Nm]
        pub fn t(&self) -> f32 {
            (self.t_s - self.t_load).clamp(0.0, self.t_s)
        }

        /// Max motor torque when having a load, using a modified base torque t_s [Unit Nm]
        pub fn t_dyn(&self, t_s : f32) -> f32 {
            (t_s - self.t_load).clamp(0.0, t_s)
        }

        /// Motor inertia when having a load [Unit kg*m^2]
        pub fn j(&self) -> f32 {
            self.j_s + self.j_load
        }

        /// Applies a certain load `j_load` to the stepper motor, which affects acceleration calculations
        pub fn apply_load_j(&mut self, j_load : f32) {
            self.j_load = j_load;
        }  

        /// Applies a certain load `j_load`
        pub fn apply_load_t(&mut self, t_load : f32) {
            self.t_load = t_load;
        }
    //

    // Conversions
        /// Converts the given angle in radians `ang` into the number of steps required to come as close as possible
        pub fn ang_to_steps_dir(&self, ang : f32) -> i64 {
            (ang / self.step_ang()).round() as i64
        }

        /// Converts the given amount of steps `steps` into an angle
        pub fn steps_to_ang_dir(&self, steps : i64) -> f32 {
            steps as f32 * self.step_ang()
        }
    //
}

pub struct ServoData
{
    /// Maximum torque of servo motor [Unit Nm]
    pub t_max : f32,

    /// Maximum angular velocity [Unit rad/s]
    pub omega_max : f32,

    // Maximum angle [Unit (radians)]
    pub phi_max : f32,

    /// Minimum signal length [Unit s]
    pub pwm_min : f32,
    /// Maximum signal length [Unit s]
    pub pwm_max : f32,

    /// Default frequency
    pub f_pwm : f32
}

impl ServoData 
{
    pub fn mg996r() -> Self {
        ServoData {
            t_max: 1.08,
            omega_max: 8.5,
            phi_max: PI,

            pwm_min: 0.001,
            pwm_max: 0.002,
            f_pwm: 50.0
        }
    }

    pub fn default(&self) -> f32 {
        (self.pwm_min + self.phi_max) / 2.0
    }

    pub fn default_pos(&self) -> f32 {
        self.phi_max / 2.0
    }

    pub fn cycle_time(&self) -> f32 {
        1.0 / self.f_pwm
    }

    pub fn pulse_time(&self, phi : f32) -> f32 {
        self.pwm_min + (self.pwm_max - self.pwm_min) / self.phi_max * phi
    }
}

// TODO: Servos