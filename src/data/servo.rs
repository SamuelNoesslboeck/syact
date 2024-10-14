use serde::{Deserialize, Serialize};

use syunit::*;

/// A struct for storing all the constants of a servo motor that do not change during the process of the program
#[derive(Debug, Default, Clone, PartialEq, PartialOrd, Serialize, Deserialize)]
pub struct ServoConst {
    /// Maximum torque of servo motor 
    pub t_max : Force,

    /// Maximum angular velocity [Unit rad/s]
    pub velocity_max : Velocity,

    /// Maximum angle [Unit (radians)]
    pub abs_pos_max : AbsPos,

    /// Minimum signal length [Unit s]
    pub pwm_min : Time,
    /// Maximum signal length [Unit s]
    pub pwm_max : Time,

    /// Default frequency
    pub f_pwm : Velocity
}

impl ServoConst {
    /// Constants for a MG996R servo motor. 
    /// See [datasheet](https://github.com/SamuelNoesslboeck/syact/blob/master/docs/datasheets/MG996R.pdf)
    pub const MG996R : Self = Self {
        t_max: Force(1.08),
        velocity_max: Velocity(8.5),
        abs_pos_max: AbsPos(core::f32::consts::PI),

        pwm_min: Time(0.00075),
        pwm_max: Time(0.00225),
        f_pwm: Velocity(50.0)
    };

    /// The default pulse time of a servo motor (mid position)
    pub fn default_pulse(&self) -> Time {
        (self.pwm_min + self.pwm_max) / 2.0
    }

    /// The default position of a servo motor
    pub fn default_pos(&self) -> AbsPos {
        self.abs_pos_max / 2.0
    }

    /// The duty cycle period time of the servo
    pub fn period_time(&self) -> Time {
        1.0 / self.f_pwm
    }

    /// Pulse time for a given `Factor`
    pub fn pulse_for_factor(&self, factor : Factor) -> Time {
        self.pwm_min + (self.pwm_max - self.pwm_min) * factor
    }

    /// Pulse time for a given angle
    pub fn pulse_for_angle(&self, abs_pos : AbsPos) -> Time {
        self.pulse_for_factor(Factor::new(abs_pos / self.abs_pos_max))
    }
}