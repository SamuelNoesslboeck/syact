use serde::{Deserialize, Serialize};

use syunit::*;

/// A struct for storing all the constants of a servo motor that do not change during the process of the program
#[derive(Debug, Default, Clone, PartialEq, PartialOrd, Serialize, Deserialize)]
pub struct ServoConst
{
    /// Maximum torque of servo motor 
    pub t_max : Force,

    /// Maximum angular velocity [Unit rad/s]
    pub velocity_max : Velocity,

    /// Maximum angle [Unit (radians)]
    pub gamma_max : Gamma,

    /// Minimum signal length [Unit s]
    pub pwm_min : Time,
    /// Maximum signal length [Unit s]
    pub pwm_max : Time,

    /// Default frequency
    pub f_pwm : Velocity
}

impl ServoConst 
{
    /// Invalid data for initialization purposes
    /// 
    /// DO NOT EXECUTE CALCULATION FUNCTIONS WITH THIS DATA
    pub const ERROR : Self = Self {
        t_max: Force::ZERO,
        velocity_max: Velocity::ZERO,
        gamma_max: Gamma::ZERO,
        pwm_min: Time::ZERO,
        pwm_max: Time::ZERO,
        f_pwm: Velocity::ZERO
    }; 

    /// Constants for a MG996R servo motor. 
    /// See [datasheet](https://github.com/SamuelNoesslboeck/syact/blob/master/docs/datasheets/MG996R.pdf)
    pub const MG996R : Self = Self {
        t_max: Force(1.08),
        velocity_max: Velocity(8.5),
        gamma_max: Gamma(core::f32::consts::PI),

        pwm_min: Time(0.00075),
        pwm_max: Time(0.00225),
        f_pwm: Velocity(50.0)
    };

    /// The default pulse time of a servo motor (mid position)
    pub fn default_pulse(&self) -> Time {
        (self.pwm_min + self.pwm_max) / 2.0
    }

    /// The default position of a servo motor
    pub fn default_pos(&self) -> Gamma {
        self.gamma_max / 2.0
    }

    /// The duty cycle period time of the servo
    pub fn period_time(&self) -> Time {
        1.0 / self.f_pwm
    }

    /// Pulse time for a given percent
    /// 
    /// # Panics
    /// 
    /// Panics if the factors is out of range (range is `0.0` to `1.0`)
    pub fn pulse_for_factor(&self, factor : f32) -> Time {
        if (1.0 < factor) & (0.0 > factor) {
            panic!("Bad factor! ({})", factor);
        }

        self.pwm_min + (self.pwm_max - self.pwm_min) * factor
    }

    /// Pulse time for a given angle
    pub fn pulse_for_angle(&self, gamma : Gamma) -> Time {
        self.pulse_for_factor(gamma / self.gamma_max)
    }
}