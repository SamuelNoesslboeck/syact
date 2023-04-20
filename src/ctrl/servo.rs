use serde::{Serialize, Deserialize};

use crate::ctrl::pwm::PWMOutput;
use crate::data::servo::ServoConst;
use crate::units::Gamma;

/// 
#[derive(Debug, Serialize, Deserialize)]
pub struct ServoDriver
{
    #[serde(skip)]
    gamma : Gamma,
    consts : ServoConst,

    pwm : PWMOutput
}

impl ServoDriver 
{
    /// Creates a new servo driver with the given servo data `consts` connected to the `pin_pwm`
    /// 
    /// Before any use of movement functions you must call `start()` in order to set up all the pins
    pub fn new(consts : ServoConst, pin_pwm : u8) -> Self {
        Self {
            gamma: consts.default_pos(),
            consts,

            pwm: PWMOutput::new(pin_pwm)
        }
    }
 
    /// Returns a reference to the `ServoConst` of the driver
    pub fn consts<'a>(&'a self) -> &'a ServoConst {
        &self.consts
    }

    /// Start the PWM signal and moves the servo to it's default position
    pub fn start(&mut self) {
        self.pwm.start();
        self.pwm.set_period(self.consts.default_pulse(), self.consts.period_time());
    }

    /// Get the *aboslute* angle of the servo 
    pub fn gamma(&self) -> Gamma {
        self.gamma
    }

    /// Set the absolute angle of the servo. Causes the servo to drive to this angle
    pub fn set_gamma(&mut self, gamma : Gamma) {
        self.pwm.set_period(self.consts.pulse_for_angle(gamma), self.consts.period_time());
        self.gamma = gamma
    }

    /// Get the duty-cycle-percent of the motor (values `0.0` to `1.0`)
    pub fn perc(&self) -> f32 {
        self.gamma / self.consts.gamma_max
    }

    /// Set the duty-cycle percent of the servo (value `0.0` to `1.0`). Causes the servo to adapt its position
    pub fn set_perc(&mut self, perc : f32) {
        self.pwm.set_period(self.consts.pulse_for_factor(perc), self.consts.period_time());
        self.gamma = self.consts.gamma_max * perc
    }

    // Positions
        /// Moves the servo to its default position
        pub fn default_pos(&mut self) {
            self.pwm.set_period(self.consts.default_pulse(), self.consts.period_time())
        }

        /// Moves the servo to its minimum endpoint
        pub fn endpoint_min(&mut self) {
            self.set_perc(0.0)
        }

        /// Moves the servo to its maximum endpoint
        pub fn endpoint_max(&mut self) {
            self.set_perc(1.0)
        }
    //

    // Drop
        /// Stops the servo driver and the PWM signal, can be started again with `start()` if desired
        pub fn stop(&mut self) {
            self.pwm.stop();
        }
    //  
}