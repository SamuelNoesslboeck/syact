use serde::{Serialize, Deserialize};

use crate::ctrl::pwm::PWMOutput;
use crate::data::servo::ServoConst;
use crate::units::Gamma;

#[derive(Debug, Serialize, Deserialize)]
pub struct ServoDriver
{
    #[serde(skip)]
    gamma : Gamma,
    #[serde(serialize_with = "ServoConst::to_standard", deserialize_with = "ServoConst::from_standard")]
    pub data : ServoConst,

    pwm : PWMOutput
}

impl ServoDriver 
{
    pub fn new(data : ServoConst, pin_pwm : u16) -> Self {
        let mut pwm = PWMOutput::spawn(pin_pwm); 
        pwm.set_period(data.default_pulse(), data.period_time());

        ServoDriver {
            gamma: data.default_pos(),
            data,

            pwm: pwm
        }
    }

    pub fn gamma(&self) -> Gamma {
        self.gamma
    }

    pub fn set_gamma(&mut self, gamma : Gamma) {
        self.pwm.set_period(self.data.pulse_for_angle(gamma), self.data.period_time());
        self.gamma = gamma
    }

    pub fn perc(&self) -> f32 {
        self.gamma / self.data.gamma_max
    }

    pub fn set_perc(&mut self, perc : f32) {
        self.pwm.set_period(self.data.pulse_for_perc(perc), self.data.period_time());
        self.gamma = self.data.gamma_max * perc
    }

    // Positions
        pub fn default_pos(&mut self) {
            self.set_perc(0.5)
        }

        pub fn endpoint_min(&mut self) {
            self.set_perc(0.0)
        }

        pub fn endpoint_max(&mut self) {
            self.set_perc(1.0)
        }
    // 
}