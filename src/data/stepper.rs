use core::f32::consts::PI;
use core::ops::Mul;
use core::str::FromStr;

use serde::{Serialize, Deserialize};
use syunit::metric::{KgMeter2, NewtonMeters};
use syunit::*;

use crate::data::ActuatorVars;

/// Microsteps used for stepper motors
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct MicroSteps(u8);

impl MicroSteps {
    /// Get the representing `u8` value
    pub fn as_u8(self) -> u8 {
        self.0
    }
}

impl From<u8> for MicroSteps {
    fn from(value: u8) -> Self {
        if (value & value.wrapping_sub(1)) == 0 {   // Check if power of 2
            Self(value)
        } else {
            panic!("Number of microsteps must be a power of 2! ({} given)", value)
        }
    }
}

impl FromStr for MicroSteps {
    type Err = <u8 as FromStr>::Err;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        Ok(Self(u8::from_str(s)?))
    }
}

impl From<MicroSteps> for u8 {
    fn from(value: MicroSteps) -> Self {
        value.0
    }
}

impl Default for MicroSteps {
    fn default() -> Self {
        Self(1)
    }
}

impl Mul<MicroSteps> for u64 {
    type Output = u64;

    fn mul(self, rhs: MicroSteps) -> Self::Output {
        self * (rhs.as_u8() as u64)
    }
}

/// Stores data for generic components 
#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct StepperConfig {
    /// Supply voltage of the components in Volts
    pub voltage : f32,

    /// Overload current of the stepper, can increase torque
    pub overload_current : Option<f32>
}

impl StepperConfig {
    /// The stepper is using 12 Volts and its rated current
    pub const VOLT12_NO_OVERLOAD : Self = Self {
        voltage: 12.0,
        overload_current: None
    }; 

    /// The stepper is using 24 Volts and its rated current
    pub const VOLT24_NO_OVERLOAD : Self = Self {
        voltage: 24.0,
        overload_current: None
    };

    /// The stepper is using 48 Volts and its rated current
    pub const VOLT48_NO_OVERLOAD : Self = Self {
        voltage: 48.0,
        overload_current: None
    };

    /// Creates a new StepperConfig instance
    #[inline(always)]
    pub fn new(voltage : f32, overload_current : Option<f32>) -> Self {
        Self { 
            voltage,
            overload_current
        }
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
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct StepperConst {
    /// Max phase current [Unit A]
    pub default_current : f32,
    /// Motor inductence [Unit H]
    pub inductance : f32,
    /// Coil resistance [Unit Ohm]
    pub resistance : f32,

    /// Step count per revolution [Unit (1)]
    pub number_steps : u64,
    /// Stall torque [Unit Nm]
    pub torque_stall : NewtonMeters,
    /// Inhertia moment [Unit kg*m^2]
    pub inertia_motor : KgMeter2
}

impl StepperConst {
    // Constants
        /// ### Stepper motor 17HE15-1504S
        /// Values for standard stepper motor, see <https://github.com/SamuelNoesslboeck/syact/docs/datasheets/17HE15_1504S.pdf>
        pub const MOT_17HE15_1504S : Self = Self {
            default_current: 1.5, 
            inductance: 0.004, 
            resistance: 2.3,
            number_steps: 200, 
            torque_stall: NewtonMeters(0.42), 
            inertia_motor: KgMeter2(0.000_005_7)
        }; 

        /// ### Stepper motor 23HS45_4204S
        /// Values for standard stepper motor, see <https://www.omc-stepperonline.com/download/23HS45-4204S.pdf>
        pub const MOT_23HS45_4204S : Self = Self {
            default_current: 3.8,
            inductance: 0.0034,
            resistance: 0.88,
            number_steps: 400,
            torque_stall: NewtonMeters(3.0),
            inertia_motor: KgMeter2(0.000_068)
        };
    // 

    // Amperage
        /// Maximum overload force with the given overload (or underload) voltage `u`
        #[inline]
        pub fn torque_overload_max(&self, voltage : f32) -> NewtonMeters {
            self.torque_stall * voltage / self.resistance / self.default_current
        }

        /// Torque created with the given overload (or underload) current `i`
        /// 
        /// ## Option
        ///
        /// Uses the default current if the given option is `None`
        #[inline]
        pub fn torque_overload(&self, current_opt : Option<f32>) -> NewtonMeters {
            self.torque_stall * current_opt.unwrap_or(self.default_current) / self.default_current
        }
    // 

    // Torque
        /// Returns the current torque [Force] that a DC-Motor can produce when driving with the 
        /// speed `velocity ` and the voltage `u` in Volts
        /// 
        /// # Panics
        /// 
        /// Panics if the given velocity is not finite
        pub fn torque_dyn(&self, mut velocity : RadPerSecond, config : &StepperConfig) -> NewtonMeters {
            velocity = velocity.abs();

            if !velocity.is_finite() {
                panic!("Bad velocity ! {}", velocity);
            }
            
            if velocity == RadPerSecond::ZERO {
                return self.torque_overload(config.overload_current);
            }

            let time = self.full_step_time(velocity);
            let pow = core::f32::consts::E.powf(time / self.tau(config.voltage));

            self.torque_overload(config.overload_current) * (pow - 1.0) / (pow + 1.0)
        }
    // 

    // Acceleration
        /// Returns the maximum acceleration that can be reached in stall
        #[inline]
        pub fn acceleration_max_stall(&self, vars : &ActuatorVars, dir : Direction) -> Option<RadPerSecond2> {
            vars.force_after_load(self.torque_stall, dir).map(|f| f / vars.inertia_after_load(self.inertia_motor))
        }

        /// Returns the maximum acceleration that can be reached 
        #[inline]
        pub fn acceleration_max_for_velocity(&self, vars : &ActuatorVars, config : &StepperConfig, velocity : RadPerSecond, dir : Direction) -> Option<RadPerSecond2> {
            vars.force_after_load(self.torque_dyn(velocity , config), dir).map(|f| f / vars.inertia_after_load(self.inertia_motor))
        }
    // 

    // U::Velocity & Inductance
        /// The inductivity constant [Unit s]
        #[inline(always)]
        pub fn tau(&self, voltage : f32) -> Seconds {
            Seconds(self.inductance * self.default_current / voltage)
        }

        /// Maximum speed for a stepper motor where it can be guarantied that it works properly
        #[inline(always)]
        pub fn velocity_max(&self, voltage : f32) -> RadPerSecond {
            RadPerSecond(PI * voltage / self.default_current / self.inductance / self.number_steps as f32)
        }

        /// Returns the start-stop-velocity for a stepper motor
        pub fn velocity_start_stop(&self, vars : &ActuatorVars, config : &StepperConfig, microsteps : MicroSteps) -> Option<RadPerSecond> {
            vars.force_after_load_lower(self.torque_overload(config.overload_current)).map(|torque| {
                RadPerSecond((torque.0 / vars.inertia_after_load(self.inertia_motor).0 * core::f32::consts::PI / (self.number_steps * microsteps) as f32).sqrt())
            })
        }
    // 

    /// U::Velocity for time per step [Unit 1/s]
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
    /// let data = StepperConst::MOT_17HE15_1504S;
    /// 
    /// assert!((data.velocity(Seconds(1.0/200.0), MicroSteps::default()) - RadPerSecond(2.0 * PI)).abs() < RadPerSecond(0.001));     
    /// ```
    #[inline(always)]
    pub fn velocity(&self, step_time : Seconds, microsteps : MicroSteps) -> RadPerSecond {
        self.step_angle(microsteps) / step_time
    }

    // Step angles & times
        /// Get the angular distance of a step in radians, considering microstepping
        /// - `micro` is the amount of microsteps per full step
        #[inline(always)]
        pub fn step_angle(&self, microsteps : MicroSteps) -> Radians {
            self.full_step_angle() / microsteps.as_u8() as f32
        }

        /// A full step angle of the motor, ignoring microstepping
        #[inline(always)]
        pub fn full_step_angle(&self) -> Radians {
            Radians(2.0 * PI / self.number_steps as f32)
        }

        /// Time per step for the given velocity 
        /// 
        /// # Unit
        /// 
        /// Returns the time in seconds
        /// 
        /// # Panics 
        /// 
        /// Panics if the given `velocity ` is zero 
        #[inline]
        pub fn step_time(&self, velocity  : RadPerSecond, microsteps : MicroSteps) -> Seconds {
            self.step_angle(microsteps) / velocity 
        }

        /// Time per full step at the given velocity velocity 
        /// 
        /// # Unit
        /// 
        /// Returns the time in seconds
        #[inline]
        pub fn full_step_time(&self, velocity  : RadPerSecond) -> Seconds {
            self.full_step_angle() / velocity 
        }
    // 

    // Steps & Angles - Conversions
        /// Converts the given angle `ang` into a absolute number of steps (always positive).
        #[inline(always)]
        pub fn steps_from_angle_abs(&self, angle : Radians, microsteps : MicroSteps) -> u64 {
            (angle.abs() / self.step_angle(microsteps)).round() as u64
        }   

        /// Converts the given angle `ang` into a number of steps
        #[inline(always)]
        pub fn steps_from_angle(&self, angle : Radians, microsteps : MicroSteps) -> i64 {
            (angle / self.step_angle(microsteps)).round() as i64
        }   

        /// Converts the given number of steps into an angle
        #[inline(always)]
        pub fn angle_from_steps_abs(&self, steps : u64, microsteps : MicroSteps) -> Radians {
            steps as f32 * self.step_angle(microsteps)
        }

        /// Converts the given number of steps into an angle
        #[inline(always)]
        pub fn angle_from_steps(&self, steps : i64, microsteps : MicroSteps) -> Radians {
            steps as f32 * self.step_angle(microsteps)
        }

        /// Rounds the given angle to the nearest step value
        #[inline(always)]
        pub fn round_angle_to_steps(&self, angle : Radians, microsteps : MicroSteps) -> Radians {
            self.angle_from_steps(self.steps_from_angle(angle, microsteps), microsteps)
        }

        // Comparision
        /// Checks wheither the given angle `ang` is in range (closes to) a given step count `steps`
        #[inline(always)]
        pub fn is_in_step_range(&self, steps : i64, angle : Radians, microsteps : MicroSteps) -> bool {
            self.steps_from_angle(angle, microsteps) == steps
        }
    //
}