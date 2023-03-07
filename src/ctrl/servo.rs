use serde::{Serialize, Deserialize};

use crate::ctrl::pwm::PWMOutput;
use crate::data::servo::ServoConst;
use crate::units::Gamma;

#[derive(Debug, Serialize, Deserialize)]
pub struct ServoDriver
{
    #[serde(skip)]
    gamma : Gamma,
    #[cfg_attr(feature = "std", serde(serialize_with = "ServoConst::to_standard", deserialize_with = "ServoConst::from_standard"))]
    pub data : ServoConst,

    pwm : PWMOutput
}

impl ServoDriver 
{
    pub fn new(data : ServoConst, pin_pwm : u8) -> Self {
        Self {
            gamma: data.default_pos(),
            data,

            pwm: PWMOutput::new(pin_pwm)
        }
    }

    pub fn start(&mut self) {
        self.pwm.start();
        self.pwm.set_period(self.data.default_pulse(), self.data.period_time());
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

    // Drop
        pub fn stop(&mut self) {
            self.pwm.stop();
        }
    //  
}