use embedded_hal::PwmPin;

use crate::{Dismantle, Setup};
use crate::device::pwm::SoftwarePWM;
use crate::units::*;

pub const LED_PWM_FREQ : Omega = Omega(200.0);

pub struct LED {
    pwm : SoftwarePWM
}

impl Setup for LED {
    fn setup(&mut self) -> Result<(), crate::Error> {
        self.pwm.setup()
    }
}

impl Dismantle for LED {
    fn dismantle(&mut self) -> Result<(), crate::Error> {
        self.pwm.dismantle()
    }
}

impl LED {
    pub fn new(pin : u8) -> Self {
        Self {
            pwm: SoftwarePWM::new(pin)
        }
    }

    // Read value
        pub fn is_on(&self) -> bool {
            self.pwm.get_duty() == self.pwm.get_max_duty()
        }

        pub fn is_off(&self) -> bool {
            !self.is_on()
        }
    // 

    // Write value
        pub fn on(&mut self) {
            self.pwm.set_freq(LED_PWM_FREQ, 1.0)
        }

        pub fn off(&mut self) {
            self.pwm.set_freq(LED_PWM_FREQ, 0.0)
        }

        pub fn set(&mut self, val : bool) {
            if val {
                self.on()
            } else {
                self.off()
            }
        }

        pub fn dim(&mut self, factor : f32) {
            self.pwm.set_freq(LED_PWM_FREQ, factor)
        }
    // 
}