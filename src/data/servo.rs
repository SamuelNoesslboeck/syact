use serde::{Deserializer, Deserialize, Serializer, Serialize};

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

    pub fn from_standard<'de, D>(deserializer: D) -> Result<Self, D::Error> 
    where 
        D: Deserializer<'de> {
        let s: String = Deserialize::deserialize(deserializer)?;
        Ok(get_standard_servo(s.as_str()).clone()) 
    }

    pub fn to_standard<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer {
        for (k, v) in &STANDARD_SERVO_CONST {
            if v == self {
                return serializer.serialize_str(*k);
            }
        }
        self.serialize(serializer)
    }

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

/// A collection of standard stepper motors
pub static STANDARD_SERVO_CONST : [(&str, ServoConst); 2] = [
    ("ERROR", ServoConst::ERROR),
    ("MG996R", ServoConst::MG996R)
];


fn get_standard_servo(name : &str) -> &ServoConst {
    for (k, v) in &STANDARD_SERVO_CONST {
        if *k == name {
            return v;
        }
    }

    &ServoConst::ERROR
}