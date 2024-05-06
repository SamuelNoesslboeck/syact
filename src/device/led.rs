use embedded_hal::pwm::SetDutyCycle;
use syunit::*;

use crate::{Dismantle, Setup};

pub const LED_PWM_FREQ : Velocity = Velocity(200.0);

pub struct LED<P : SetDutyCycle> {
    pwm : P
}

impl<P : SetDutyCycle + Setup> Setup for LED<P> {
    fn setup(&mut self) -> Result<(), crate::Error> {
        self.pwm.setup()
    }
}

impl<P : SetDutyCycle + Dismantle> Dismantle for LED<P> {
    fn dismantle(&mut self) -> Result<(), crate::Error> {
        self.pwm.dismantle()
    }
}

impl<P : SetDutyCycle> LED<P> {
    pub fn new(pwm : P) -> Self {
        Self {
            pwm
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
            self.pwm.set_duty_cycle_fully_on()
        }

        pub fn off(&mut self) {
            self.pwm.set_duty_cycle_fully_off()
        }

        pub fn set(&mut self, val : bool) {
            if val {
                self.on()
            } else {
                self.off()
            }
        }

        pub fn dim(&mut self, factor : f32) {
            self
        }
    // 
}