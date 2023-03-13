//! ### Data subfile
//! 
//! Structs for storing stepper motor data and performing basic calculations \
//! NOTE: If any of the formulas are unclear, please have a look at \
//! the article on stepper motors included in this create under "article/article.pdf"

use core::f32::consts::PI;

use serde::{Serialize, Deserialize};

#[cfg(feature = "std")]
use serde::{Deserializer, Serializer};

// Submodules
mod lk;
pub use lk::LinkedData;

/// Crate for servo motor data
pub mod servo;

/// Crate for variables read and written during runtime
mod var;
pub use var::CompVars;

use crate::units::*;
//

/**
### `StepperData`
A collection of the most relevant variables Unit stepper calculation 
```
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
    pub t_s : Force,
    /// Inhertia moment [Unit kg*m^2]
    pub j_s : Inertia
}

impl StepperConst
{
    /// Error stepperdata with all zeros
    pub const ERROR : Self = Self {
        i_max: 0.0,
        l: 0.0,
        n_s: 0,
        n_c: 0,
        t_s: Force::ZERO,
        j_s: Inertia::ZERO
    }; 

    pub const GEN : Self = Self::MOT_17HE15_1504S;

    /// ### Stepper motor 17HE15-1504S
    /// Values for standard stepper motor \
    /// Link: <https://www.omc-stepperonline.com/de/e-serie-nema-17-bipolar-42ncm-59-49oz-in-1-5a-42x42x38mm-4-draehte-w-1m-kabel-verbinder-17he15-1504s>
    pub const MOT_17HE15_1504S : Self = Self {
        i_max: 1.5, 
        l: 0.004, 
        n_s: 200, 
        n_c: 100,
        t_s: Force(0.42), 
        j_s: Inertia(0.000_005_7)
    }; 
    
    // TODO: Rework
    #[cfg(feature = "std")]
    pub fn from_standard<'de, D>(deserializer: D) -> Result<Self, D::Error> 
    where 
        D: Deserializer<'de> {
        let s: String = Deserialize::deserialize(deserializer)?;
        Ok(get_standard_mot(s.as_str()).clone()) 
    }

    #[cfg(feature = "std")]
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

    /// The maximum angular acceleration of the motor (in stall) in consideration of the current loads
    #[inline(always)]
    pub fn alpha_max(&self, var : &CompVars) -> Result<Alpha, crate::Error> {
        Ok(self.t(var.t_load)? / self.j(var.j_load))
    }

    /// The maximum angular acceleration of the motor, with a modified torque t_s
    #[inline(always)]
    pub fn alpha_max_dyn(&self, t_s : Force, var : &CompVars) -> Result<Alpha, crate::Error> {
        Ok(Self::t_dyn(t_s, var.t_load)? / self.j(var.j_load))
    }

    /// The inductivity constant [Unit s]
    #[inline(always)]
    pub fn tau(&self, u : f32) -> Time {
        Time(self.i_max * self.l / u)
    }

    /// Omega for time per step [Unit 1/s]
    /// 
    /// # Panics
    /// 
    /// Panics if the given `step_time` is zero (`-0.0` included)
    /// 
    /// ```rust 
    /// use core::f32::consts::PI;
    /// 
    /// use stepper_lib::data::StepperConst;
    /// use stepper_lib::units::*;
    /// 
    /// let data = StepperConst::GEN;
    /// 
    /// assert!((data.omega(Time(1.0/200.0)) - Omega(2.0 * PI)).abs() < Omega(0.001));     
    /// ```
    #[inline(always)]
    pub fn omega(&self, step_time : Time) -> Omega {
        if (step_time == Time(0.0)) | (step_time == Time(-0.0)) {
            panic!("The given step time ({}) is zero!", step_time)
        }

        self.step_ang() / step_time
    }

    // Steps
        /// Get the angular distance of a step Unit rad [Unit 1]
        #[inline(always)]
        pub fn step_ang(&self) -> Delta {
            Delta(2.0 * PI / self.n_s as f32)
        }

        /// Time per step for the given omega [Unit s]
        /// 
        /// # Panics 
        /// 
        /// Panics if the given `omega` is zero 
        #[inline(always)]
        pub fn step_time(&self, omega : Omega) -> Time {
            if (omega == Omega(0.0)) | (omega == Omega(-0.0)) {
                panic!("The given omega ({}) is zero!", omega);
            }

            2.0 * PI / (self.n_s as f32) / omega
        }
    // 

    // Load calculations
        /// Max motor torque when having a load [Unit Nm]
        /// 
        /// # Pancis
        /// 
        /// 
        #[inline(always)]
        pub fn t(&self, t_load : Force) -> Result<Force, crate::Error> {  // TODO: Add overload protection
            if !t_load.is_finite() {
                panic!("The given load force ({}) is invalid!", t_load);
            }

            if t_load > self.t_s {
                Err(crate::Error::new(std::io::ErrorKind::InvalidInput, 
                    format!("Overload! (Motor torque: {}, Load: {})", self.t_s, t_load)))
            } else {
                Ok(self.t_s - t_load)
            }
        }

        /// Max motor torque when having a load, using a modified base torque t_s [Unit Nm]
        /// 
        /// # Panics 
        /// 
        /// Panics if the given motor torque `t_s` is negative (-0.0 included), infinite or NAN
        #[inline(always)]
        pub fn t_dyn(t_s : Force, t_load : Force) -> Result<Force, crate::Error> { // TODO: Add overload protection
            if t_s.is_sign_negative() | (!t_s.is_finite()) {
                panic!("The given force ({}) is invalid!", t_s);
            }

            if !t_load.is_finite() {
                panic!("The given load force ({}) is invalid!", t_load);
            }

            if t_load > t_s {
                Err(crate::Error::new(std::io::ErrorKind::InvalidInput, 
                    format!("Overload! (Motor torque: {}, Load: {})", t_s, t_load)))
            } else {
                Ok(t_s - t_load)
            }
        }

        /// Motor inertia when having a load [Unit kg*m^2]
        /// 
        /// # Panics
        /// 
        /// Panics if the given inertia `j_load` is negative (-0.0 included)
        #[inline(always)]
        pub fn j(&self, j_load : Inertia) -> Inertia {
            if j_load.is_sign_negative() | (!j_load.is_finite()) {
                panic!("The given inertia ({}) is invalid!", j_load);
            }

            self.j_s + j_load
        }
    //

    // Conversions
        #[inline(always)]
        pub fn steps_from_ang(&self, ang : Delta) -> u64 {
            (ang.abs() / self.step_ang()).round() as u64
        }

        #[inline(always)]
        pub fn steps_from_ang_dir(&self, ang : Delta) -> i64 {
            (ang / self.step_ang()).round() as i64
        }   

        #[inline(always)]
        pub fn ang_from_steps(&self, steps : u64) -> Delta {
            steps as f32 * self.step_ang()
        }

        #[inline(always)]
        pub fn ang_from_steps_dir(&self, steps : i64) -> Delta {
            steps as f32 * self.step_ang()
        }
    //
}

/// A collection of standard stepper motors
pub static STANDARD_STEPPER_CONST : [(&str, StepperConst); 2] = [
    ("ERROR", StepperConst::ERROR),
    ("MOT_17HE15_1504S", StepperConst::MOT_17HE15_1504S)
];

#[cfg(feature = "std")]
fn get_standard_mot(name : &str) -> &StepperConst {
    for (k, v) in &STANDARD_STEPPER_CONST {
        if *k == name {
            return v;
        }
    }

    &StepperConst::ERROR
}