#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

use syunit::*;
use syunit::metric::*;

/// A struct for storing all the constants of a servo motor that do not change during the process of the program
#[derive(Debug, Default, Clone, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ServoConst {
    /// Maximum torque of servo motor 
    pub t_max : NewtonMeters,

    /// Maximum angular velocity [Unit rad/s]
    pub velocity_max : RadPerSecond,

    /// Maximum angle [Unit radians]
    pub position_max : PositionRad,

    /// Duty at minimum position
    pub duty_min : Factor,
    /// Duty at maximum position
    pub duty_max : Factor,

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

        duty_min: unsafe { Factor::new_unchecked(0.05) },
        duty_max: unsafe { Factor::new_unchecked(0.1) },
        f_pwm: Hertz(50.0)
    };

    /// The default duty of a servo in default (mid) position
    pub fn default_duty(&self) -> Factor {
        Factor::new((self.duty_min.as_f32() + self.duty_max.as_f32()) / 2.0)
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
    pub fn duty_for_factor(&self, factor : Factor) -> Factor {
        Factor::new(self.duty_min.as_f32() + (self.duty_max.as_f32() - self.duty_min.as_f32()) * factor.as_f32())
    }

    /// Pulse time for a given angle
    pub fn duty_for_pos(&self, pos : PositionRad) -> Factor {
        self.duty_for_factor(Factor::new(pos / self.position_max))
    }
}