use core::f32::consts::PI;

use serde::{Serialize, Deserialize};
use sylo::Direction;

use crate::data::MicroSteps;
use crate::math::force::torque_dyn;
use crate::{units::*, ActuatorVars};

/// Stores data for generic components 
#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct StepperConfig {
    /// Supply voltage of the components in Volts
    pub voltage : f32
}

impl StepperConfig {
    /// Generic `StepperConfig` for testing purposes
    pub const GEN : Self = Self {
        voltage: 12.0
    };  

    /// Comp data that will case an error in calculations as it has not been yet initialized.  
    /// 
    /// (Has to be overwritten, will cause errors otherwise)
    pub const ERROR : Self = Self {
        voltage: 0.0
    };

    /// Creates a new StepperConfig instance
    /// 
    /// # Panics
    /// 
    /// Panics if the given safety factor is smaller than 1.0
    #[inline(always)]
    pub fn new(voltage : f32) -> Self {
        Self { voltage }
    }
}

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
pub struct StepperConst {
    /// Max phase current [Unit A]
    pub current_max : f32,
    /// Motor inductence [Unit H]
    pub inductance : f32,
    /// Coil resistance [Unit Ohm]
    pub resistance : f32,

    /// Step count per revolution [Unit (1)]
    pub number_steps : u64,
    /// Stall torque [Unit Nm]
    pub torque_stall : Force,
    /// Inhertia moment [Unit kg*m^2]
    pub inertia_motor : Inertia
}

impl StepperConst {
    // Constants
        /// Error stepperdata with all zeros
        pub const ERROR : Self = Self {
            current_max: 0.0,
            inductance: 0.0,
            resistance: 0.0,
            number_steps: 0,
            torque_stall: Force::ZERO,
            inertia_motor: Inertia::ZERO
        }; 

        /// Generic stepper motor data, only in use when testing for simple syntax
        pub const GEN : Self = Self::MOT_17HE15_1504S;

        /// ### Stepper motor 17HE15-1504S
        /// Values for standard stepper motor, see <https://github.com/SamuelNoesslboeck/syact/docs/datasheets/17HE15_1504S.pdf>
        pub const MOT_17HE15_1504S : Self = Self {
            current_max: 1.5, 
            inductance: 0.004, 
            resistance: 2.3,
            number_steps: 200, 
            torque_stall: Force(0.42), 
            inertia_motor: Inertia(0.000_005_7)
        }; 
    // 

    // Amperage
        /// Maximum overload force with the given overload (or underload) voltage `u`
        pub fn torque_overload_max(&self, voltage : f32) -> Force {
            self.torque_stall * voltage / self.resistance / self.current_max
        }

        /// Torque created with the given overload (or underload) current `i`
        pub fn torque_overload(&self, current : f32) -> Force {
            self.torque_stall * current / self.current_max
        }
    // 

    // Acceleration
        pub fn alpha_max_stall(&self, vars : &ActuatorVars, dir : Direction) -> Option<Alpha> {
            vars.force_after_load(self.torque_stall, dir).map(|f| f / vars.inertia_after_load(self.inertia_motor))
        }

        pub fn alpha_max_for_omega(&self, vars : &ActuatorVars, config : &StepperConfig, omega : Omega, dir : Direction) -> Option<Alpha> {
            vars.force_after_load(torque_dyn(self, omega, config.voltage, self.current_max), dir).map(|f| f / vars.inertia_after_load(self.inertia_motor))
        }
    // 

    // Speeds
        /// The inductivity constant [Unit s]
        #[inline(always)]
        pub fn tau(&self) -> Time {
            Time(self.inductance / self.resistance)
        }

        /// Maximum speed for a stepper motor where it can be guarantied that it works properly
        #[inline(always)]
        pub fn omega_max(&self, voltage : f32) -> Omega {
            Omega(2.0 * PI * voltage / self.current_max / self.inductance / self.number_steps as f32)
        }
    // 

    /// Omega for time per step [Unit 1/s]
    /// 
    /// # Panics
    /// 
    /// Panics if the given `step_time` is zero (`-0.0` included)
    /// 
    /// ```rust 
    /// use core::f32::consts::PI;
    /// 
    /// use syact::prelude::*;
    /// 
    /// let data = StepperConst::GEN;
    /// 
    /// assert!((data.omega(Time(1.0/200.0), MicroSteps::default()) - Omega(2.0 * PI)).abs() < Omega(0.001));     
    /// ```
    #[inline(always)]
    pub fn omega(&self, step_time : Time, microsteps : MicroSteps) -> Omega {
        if (step_time == Time(0.0)) | (step_time == Time(-0.0)) {
            panic!("The given step time ({}) is zero!", step_time)
        }

        self.step_angle(microsteps) / step_time
    }

    // Step angles & times
        /// Get the angular distance of a step in radians, considering microstepping
        /// - `micro` is the amount of microsteps per full step
        #[inline(always)]
        pub fn step_angle(&self, microsteps : MicroSteps) -> Delta {
            self.full_step_angle() / microsteps.as_u8() as f32
        }

        /// A full step angle of the motor, ignoring microstepping
        #[inline(always)]
        pub fn full_step_angle(&self) -> Delta {
            Delta(2.0 * PI / self.number_steps as f32)
        }

        /// Time per step for the given omega
        /// 
        /// # Unit
        /// 
        /// Returns the time in seconds
        /// 
        /// # Panics 
        /// 
        /// Panics if the given `omega` is zero 
        #[inline]
        pub fn step_time(&self, omega : Omega, microsteps : MicroSteps) -> Time {
            // if (omega == Omega(0.0)) | (omega == Omega(-0.0)) {
            //     panic!("The given omega ({}) is zero!", omega);
            // }

            self.step_angle(microsteps) / omega
        }

        /// Time per full step at the given velocity omega
        /// 
        /// # Unit
        /// 
        /// Returns the time in seconds
        #[inline]
        pub fn full_step_time(&self, omega : Omega) -> Time {
            if (omega == Omega(0.0)) | (omega == Omega(-0.0)) {
                panic!("The given omega ({}) is zero!", omega);
            }

            self.full_step_angle() / omega
        }
    // 

    // Steps & Angles - Conversions
        /// Converts the given angle `ang` into a absolute number of steps (always positive).
        #[inline(always)]
        pub fn steps_from_angle_abs(&self, angle : Delta, microsteps : MicroSteps) -> u64 {
            (angle.abs() / self.step_angle(microsteps)).round() as u64
        }   

        /// Converts the given angle `ang` into a number of steps
        #[inline(always)]
        pub fn steps_from_angle(&self, angle : Delta, microsteps : MicroSteps) -> i64 {
            (angle / self.step_angle(microsteps)).round() as i64
        }   

        /// Converts the given number of steps into an angle
        #[inline(always)]
        pub fn angle_from_steps_abs(&self, steps : u64, microsteps : MicroSteps) -> Delta {
            steps as f32 * self.step_angle(microsteps)
        }

        /// Converts the given number of steps into an angle
        #[inline(always)]
        pub fn angle_from_steps(&self, steps : i64, microsteps : MicroSteps) -> Delta {
            steps as f32 * self.step_angle(microsteps)
        }

        // Comparision
        /// Checks wheither the given angle `ang` is in range (closes to) a given step count `steps`
        #[inline(always)]
        pub fn is_in_step_range(&self, steps : i64, angle : Delta, microsteps : MicroSteps) -> bool {
            self.steps_from_angle(angle, microsteps) == steps
        }
    //
}