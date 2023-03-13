use serde::{Deserialize, Serialize};

use crate::units::*;

#[derive(Debug, Default, Clone, PartialEq, PartialOrd, Serialize, Deserialize)]
pub struct ServoConst
{
    /// Maximum torque of servo motor 
    pub t_max : Force,

    /// Maximum angular velocity [Unit rad/s]
    pub omega_max : Omega,

    // Maximum angle [Unit (radians)]
    pub gamma_max : Gamma,

    /// Minimum signal length [Unit s]
    pub pwm_min : Time,
    /// Maximum signal length [Unit s]
    pub pwm_max : Time,

    /// Default frequency
    pub f_pwm : Omega
}

impl ServoConst 
{
    pub const ERROR : Self = Self {
        t_max: Force::ZERO,
        omega_max: Omega::ZERO,
        gamma_max: Gamma::ZERO,
        pwm_min: Time::ZERO,
        pwm_max: Time::ZERO,
        f_pwm: Omega::ZERO
    }; 

    pub const MG996R : Self = Self {
        t_max: Force(1.08),
        omega_max: Omega(8.5),
        gamma_max: Gamma(core::f32::consts::PI),

        pwm_min: Time(0.00075),
        pwm_max: Time(0.00225),
        f_pwm: Omega(50.0)
    };

    pub fn default_pulse(&self) -> Time {
        (self.pwm_min + self.pwm_max) / 2.0
    }

    pub fn default_pos(&self) -> Gamma {
        self.gamma_max / 2.0
    }

    pub fn period_time(&self) -> Time {
        1.0 / self.f_pwm
    }

    pub fn pulse_for_perc(&self, perc : f32) -> Time {
        self.pwm_min + (self.pwm_max - self.pwm_min) * perc.clamp(0.0, 1.0)
    }

    pub fn pulse_for_angle(&self, gamma : Gamma) -> Time {
        self.pulse_for_perc(gamma / self.gamma_max)
    }
}
