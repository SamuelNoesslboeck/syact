use core::f32::consts::PI;

use serde::{Serialize, Deserialize};

use crate::{units::*, lib_error};

// Submodules
mod comp;
pub use comp::CompData;

/// Crate for servo motor data
pub mod servo;

/// Crate for variables read and written during runtime
mod var;
pub use var::CompVars;
//

/// A collection of the most relevant variables Unit stepper calculation 
/// ```
/// use syact::StepperConst;
///
/// // Create the data from an standard motor
/// let mut data = StepperConst::MOT_17HE15_1504S;
///
/// ``` 
/// Supports JSON-Serialize and Deserialize with the `serde_json` library
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct StepperConst
{
    /// Max phase current [Unit A]
    pub i_max : f32,
    /// Motor inductence [Unit H]
    pub l : f32,

    /// Step count per revolution [Unit (1)]
    pub n_s : u64,
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
        t_s: Force::ZERO,
        j_s: Inertia::ZERO
    }; 

    /// Generic stepper motor data, only in use when testing for simple syntax
    pub const GEN : Self = Self::MOT_17HE15_1504S;

    /// ### Stepper motor 17HE15-1504S
    /// Values for standard stepper motor, see <https://github.com/SamuelNoesslboeck/syact/docs/datasheets/17HE15_1504S.pdf>
    pub const MOT_17HE15_1504S : Self = Self {
        i_max: 1.5, 
        l: 0.004, 
        n_s: 200, 
        t_s: Force(0.42), 
        j_s: Inertia(0.000_005_7)
    }; 

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
        Time(2.0 * self.i_max * self.l / u)
    }

    /// Maximum speed for a stepper motor where it can be guarantied that it works properly
    #[inline(always)]
    pub fn max_speed(&self, u : f32) -> Omega {
        2.0 * PI / self.tau(u) / self.n_s as f32
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
    /// use syact::data::StepperConst;
    /// use syact::units::*;
    /// 
    /// let data = StepperConst::GEN;
    /// 
    /// assert!((data.omega(Time(1.0/200.0), 1) - Omega(2.0 * PI)).abs() < Omega(0.001));     
    /// ```
    #[inline(always)]
    pub fn omega(&self, step_time : Time, micro : u8) -> Omega {
        if (step_time == Time(0.0)) | (step_time == Time(-0.0)) {
            panic!("The given step time ({}) is zero!", step_time)
        }

        self.step_ang(micro) / step_time
    }

    // Steps
        /// Get the angular distance of a step Unit rad [Unit 1]
        #[inline(always)]
        pub fn step_ang(&self, micro : u8) -> Delta {
            Delta(2.0 * PI / self.n_s as f32 / micro as f32)
        }

        /// Time per step for the given omega [Unit s]
        /// 
        /// # Panics 
        /// 
        /// Panics if the given `omega` is zero 
        #[inline(always)]
        pub fn step_time(&self, omega : Omega, micro : u8) -> Time {
            if (omega == Omega(0.0)) | (omega == Omega(-0.0)) {
                panic!("The given omega ({}) is zero!", omega);
            }

            self.step_ang(micro) / omega
        }
    // 

    // Load calculations
        /// Max motor torque when having a load [Unit Nm]
        /// 
        /// # Errors
        /// 
        /// Returns an error if the load torque `t_load` is larger than the stall torque `t_s`
        /// 
        /// # Pancis
        /// 
        /// Panics if the load force is either
        #[inline(always)]
        pub fn t(&self, t_load : Force) -> Result<Force, crate::Error> {  // TODO: Add overload protection
            if !t_load.is_finite() {
                panic!("The given load force ({}) is invalid!", t_load);
            }

            #[cfg(feature = "std")]
            if t_load > self.t_s {
                Err(lib_error(format!("Overload! (Motor torque: {}, Load: {})", self.t_s, t_load)))
            } else {
                Ok(self.t_s - t_load)
            }

            #[cfg(not(feature = "std"))]
            if t_load > self.t_s {
                Err(crate::ErrorKind::Overload)
            } else {
                Ok(self.t_s - t_load)
            }
        }

        /// Max motor torque when having a load, using a modified base torque t_s [Unit Nm]
        /// 
        /// # Errors
        /// 
        /// Returns an error if the load torque `t_load` is larger than the stall torque `t_s`
        /// 
        /// # Panics 
        /// 
        /// Panics if either the given motor torque `t_s` or load torque `t_load` are negative, zero (-0.0 included), infinite or NAN
        /// 
        /// ```rust
        /// use syact::data::StepperConst;
        /// use syact::units::*;
        /// 
        /// // Base torque of the motor
        /// const BASE : Force = Force(1.0);
        /// 
        /// // Load torque
        /// const LOAD_1 : Force = Force(0.5);
        /// const LOAD_2 : Force = Force(1.5);
        /// 
        /// // Compare
        /// assert_eq!(StepperConst::t_dyn(BASE, LOAD_1).unwrap(), Force(0.5));
        /// assert!(StepperConst::t_dyn(BASE, LOAD_2).is_err());
        /// ```
        #[inline(always)]
        pub fn t_dyn(t_s : Force, t_load : Force) -> Result<Force, crate::Error> { // TODO: Add overload protection
            if !t_s.is_normal() {
                panic!("The given motor force ({}) is invalid!", t_load);
            }

            if !t_load.is_finite() {
                panic!("The given load force ({}) is invalid!", t_load);
            }

            #[cfg(feature = "std")]
            if t_load > t_s {
                Err(lib_error(format!("Overload! (Motor torque: {}, Load: {})", t_s, t_load)))
            } else {
                Ok(t_s - t_load)
            }

            #[cfg(not(feature = "std"))]
            if t_load > t_s {
                Err(crate::ErrorKind::Overload)
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
        /// Converts the given angle `ang` into a absolute number of steps (always positive).
        #[inline(always)]
        pub fn steps_from_ang_abs(&self, ang : Delta, micro : u8) -> u64 {
            (ang.abs() / self.step_ang(micro)).round() as u64
        }   

        /// Converts the given angle `ang` into a number of steps
        #[inline(always)]
        pub fn steps_from_ang(&self, ang : Delta, micro : u8) -> i64 {
            (ang / self.step_ang(micro)).round() as i64
        }   

        /// Converts the given number of steps into an angle
        #[inline(always)]
        pub fn ang_from_steps_abs(&self, steps : u64, micro : u8) -> Delta {
            steps as f32 * self.step_ang(micro)
        }

        /// Converts the given number of steps into an angle
        #[inline(always)]
        pub fn ang_from_steps(&self, steps : i64, micro : u8) -> Delta {
            steps as f32 * self.step_ang(micro)
        }

        // Comparision
        /// Checks wheither the given angle `ang` is in range (closes to) a given step count `steps`
        #[inline(always)]
        pub fn is_in_step_range(&self, steps : i64, ang : Delta, micro : u8) -> bool {
            self.steps_from_ang(ang, micro) == steps
        }
    //
}
