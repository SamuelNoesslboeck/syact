use syunit::*;
use syunit::metric::*;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Configuration used for the servo motor
#[derive(Debug, Clone)]
pub struct MiniServoConfig {
    /// Voltage that is applied to the servo motor
    pub voltage : f32
}

/// A struct for storing all the constants of a servo motor that do not change during the process of the program
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MiniServoData {
    /* Voltage dep. */
        /// Maximum torque of servo motor 
        pub torque_stall : NewtonMeters,
        /// Angular velocity at the default voltage
        pub velocity_free : RadPerSecond,
        /// Default voltage for the motors operation, `torque_stall` and `velocity_free` reference this
        pub voltage_default : f32,
        /// Voltage range at which the motor can be operated
        pub voltage_range : (f32, f32),
    /**/

    /// Maximum angle, minimum angle always starts counting at zero
    pub position_max : PositionRad,

    /// Duty at minimum position
    pub duty_min : Factor,
    /// Duty at maximum position
    pub duty_max : Factor
}

impl MiniServoData {
    /* Commmon Motors */
        /// Constants for a MG996R servo motor
        pub const MG996R : Self = Self {
            torque_stall: NewtonMeters(1.08),
            velocity_free: RadPerSecond(7.48),

            voltage_default: 6.0,
            voltage_range: (4.8, 7.2),

            position_max: PositionRad(core::f32::consts::PI),

            duty_min: unsafe { Factor::new_unchecked(0.025) },
            duty_max: unsafe { Factor::new_unchecked(0.125) }
        };
    /**/

    /* Duty calculations */
        /// Pulse time for a given [Factor], with `0.0` being minimum position and `1.0` being maximum position
        pub fn duty_for_factor(&self, factor : Factor) -> Factor {
            Factor::new(self.duty_min.as_f32() + (self.duty_max.as_f32() - self.duty_min.as_f32()) * factor.as_f32())
        }

        /// Pulse time for a given angle
        pub fn duty_for_pos(&self, pos : PositionRad) -> Factor {
            self.duty_for_factor(Factor::new(pos / self.position_max))
        }
    /**/


}