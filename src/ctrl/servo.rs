use serde::{Serialize, Deserialize};

use crate::ctrl::pwm::PWMOutput;
use crate::data::servo::ServoConst;
use crate::units::Gamma;

#[derive(Debug, Serialize, Deserialize)]
pub struct ServoDriver
{
    #[serde(skip)]
    gamma : Gamma,
    pub consts : ServoConst,

    pwm : PWMOutput
}

impl ServoDriver 
{
    pub fn new(consts : ServoConst, pin_pwm : u8) -> Self {
        Self {
            gamma: consts.default_pos(),
            consts,

            pwm: PWMOutput::new(pin_pwm)
        }
    }

    pub fn start(&mut self) {
        self.pwm.start();
        self.pwm.set_period(self.consts.default_pulse(), self.consts.period_time());
    }

    pub fn gamma(&self) -> Gamma {
        self.gamma
    }

    pub fn set_gamma(&mut self, gamma : Gamma) {
        self.pwm.set_period(self.consts.pulse_for_angle(gamma), self.consts.period_time());
        self.gamma = gamma
    }

    pub fn perc(&self) -> f32 {
        self.gamma / self.consts.gamma_max
    }

    pub fn set_perc(&mut self, factor : f32) {
        self.pwm.set_period(self.consts.pulse_for_factor(factor), self.consts.period_time());
        self.gamma = self.consts.gamma_max * factor
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