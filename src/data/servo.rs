use serde::{Deserialize, Serialize};

use syunit::*;
use syunit::metric::*;

/// A struct for storing all the constants of a servo motor that do not change during the process of the program
#[derive(Debug, Default, Clone, PartialEq, PartialOrd, Serialize, Deserialize)]
pub struct ServoConst {
    /// Maximum torque of servo motor 
    pub t_max : NewtonMeters,

    /// Maximum angular velocity [Unit rad/s]
    pub velocity_max : RadPerSecond,

    /// Maximum angle [Unit (radians)]
    pub position_max : PositionRad,

    /// Minimum signal length [Unit s]
    pub pwm_min : Seconds,
    /// Maximum signal length [Unit s]
    pub pwm_max : Seconds,

    /// Default frequency
    pub f_pwm : Hertz
}

impl ServoConst {
    /// Constants for a MG996R servo motor. 
    /// See [datasheet](https://github.com/SamuelNoesslboeck/syact/blob/master/docs/datasheets/MG996R.pdf)
    pub const MG996R : Self = Self {
        t_max: NewtonMeters(1.08),
        velocity_max: RadPerSecond(8.5),
        position_max: PositionRad(core::f32::consts::PI),

        pwm_min: Seconds(0.00075),
        pwm_max: Seconds(0.00225),
        f_pwm: Hertz(50.0)
    };

    /// The default pulse time of a servo motor (mid position)
    pub fn default_pulse(&self) -> Seconds {
        (self.pwm_min + self.pwm_max) / 2.0
    }

    /// The default position of a servo motor
    pub fn default_pos(&self) -> PositionRad {
        self.position_max / 2.0
    }

    /// The duty cycle period time of the servo
    pub fn period_time(&self) -> Seconds {
        1.0 / self.f_pwm
    }

    /// Pulse time for a given [Factor]
    pub fn pulse_for_factor(&self, factor : Factor) -> Seconds {
        self.pwm_min + (self.pwm_max - self.pwm_min) * factor
    }

    /// Pulse time for a given angle
    pub fn pulse_for_angle(&self, pos : PositionRad) -> Seconds {
        self.pulse_for_factor(Factor::new(pos / self.position_max))
    }
}