use embedded_hal::pwm::SetDutyCycle;
use syunit::*;

use crate::{Dismantle, Setup};

pub const LED_PWM_FREQ : Velocity = Velocity(200.0);

pub struct LED<P : SetDutyCycle> {
    pwm : P,
    duty : u16
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
            pwm,
            duty: 0
        }
    }

    // Read value
        pub fn is_on(&self) -> bool {
            self.duty == self.pwm.max_duty_cycle()
        }

        pub fn is_off(&self) -> bool {
            !self.is_on()
        }

        pub fn dim_fac(&self) -> Factor {
            // Safe to use here
            unsafe { Factor::new_unchecked((self.duty as f32) / (self.pwm.max_duty_cycle() as f32)) }
        }
    // 

    // Write value
        pub fn on(&mut self) -> Result<(), P::Error> {
            self.pwm.set_duty_cycle_fully_on().map(|v| {
                self.duty = self.pwm.max_duty_cycle();
                v
            })
        }

        pub fn off(&mut self) -> Result<(), P::Error> {
            self.pwm.set_duty_cycle_fully_off().map(|v| {
                self.duty = 0;
                v
            })
        }

        pub fn set(&mut self, val : bool) -> Result<(), P::Error> {
            if val {
                self.on()
            } else {
                self.off()
            }
        }

        pub fn dim(&mut self, factor : Factor) -> Result<(), P::Error> {
            self.pwm.set_duty_cycle(factor.get_duty_for(self.pwm.max_duty_cycle()))
        }
    // 
}