//! ### Data subfile
//! 
//! Structs for storing stepper motor data and performing basic calculations \
//! NOTE: If any of the formulas are unclear, please have a look at \
//! the article on stepper motors included in this create under "article/article.pdf"

use std::f32::consts::PI;

use serde::{Serialize, Deserialize, Deserializer, Serializer};

// Submodules
pub mod servo;
pub use servo::*;
//

/**
### `StepperData`
A collection of the most relevant variables Unit stepper calculation 
```rust
use stepper_lib::StepperConst;

// Create the data from an standard motor
let mut data = StepperConst::MOT_17HE15_1504S;

``` 
Supports JSON-Serialize and Deserialize with the `serde_json` library
*/
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct StepperConst
{
    /// Max phase current [Unit A]
    pub i_max : f32,
    /// Motor inductence [Unit H]
    pub l : f32,

    /// Step count per revolution [Unit (1)]
    pub n_s : u64,
    /// Coil pair count (n_s / 2) Unit many cases [Unit (1)]
    pub n_c : u64,
    /// Stall torque [Unit Nm]
    pub t_s : f32,
    /// Inhertia moment [Unit kg*m^2]
    pub j_s : f32,
}

impl StepperConst
{
    pub const ERROR : Self = Self {
        i_max: 0.0,
        l: 0.0,
        n_s: 0,
        n_c: 0,
        t_s: 0.0,
        j_s: 0.0
    }; 

    /// ### Stepper motor 17HE15-1504S
    /// Values for standard stepper motor \
    /// Link: https://www.omc-stepperonline.com/de/e-serie-nema-17-bipolar-42ncm-59-49oz-in-1-5a-42x42x38mm-4-draehte-w-1m-kabel-verbinder-17he15-1504s
    pub const MOT_17HE15_1504S : Self = Self {
        i_max: 1.5, 
        l: 0.004, 
        n_s: 200, 
        n_c: 100,
        t_s: 0.42, 
        j_s: 0.000_005_7
    }; 

    pub fn from_standard<'de, D>(deserializer: D) -> Result<Self, D::Error> 
    where 
        D: Deserializer<'de> {
        let s: String = Deserialize::deserialize(deserializer)?;
        Ok(get_standard_mot(s.as_str()).clone()) 
    }

    pub fn to_standard<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer {
        for (k, v) in &STANDARD_STEPPER_CONST {
            if v == self {
                return serializer.serialize_str(*k);
            }
        }
        self.serialize(serializer)
    }

    /// The maximum angular acceleration of the motor (in stall) [Unit s^-2]
    pub fn alpha_max(&self, t_load : f32, j_load : f32) -> f32 {
        self.t(t_load) / self.j(j_load)
    }

    /// The maximum angular acceleration of the motor, with a modified torque t_s [Unit s^-2]
    pub fn alpha_max_dyn(&self, t_s : f32, t_load : f32, j_load : f32) -> f32 {
        Self::t_dyn(t_s, t_load) / self.j(j_load)
    }

    /// The inductivity constant [Unit s]
    pub fn tau(&self, u : f32) -> f32 {
        self.i_max * self.l / u
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
        pub fn t(&self, t_load : f32) -> f32 {
            (self.t_s - t_load).clamp(0.0, self.t_s)
        }

        /// Max motor torque when having a load, using a modified base torque t_s [Unit Nm]
        pub fn t_dyn(t_s : f32, t_load : f32) -> f32 {
            (t_s - t_load).clamp(0.0, t_s)
        }

        /// Motor inertia when having a load [Unit kg*m^2]
        pub fn j(&self, j_load : f32) -> f32 {
            self.j_s + j_load
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

pub static STANDARD_STEPPER_CONST : [(&str, StepperConst); 2] = [
    ("ERROR", StepperConst::ERROR),
    ("MOT_17HE15_1504S", StepperConst::MOT_17HE15_1504S)
];

fn get_standard_mot(name : &str) -> &StepperConst {
    for (k, v) in &STANDARD_STEPPER_CONST {
        if *k == name {
            return v;
        }
    }

    &StepperConst::ERROR
}