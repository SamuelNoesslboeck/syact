use embedded_hal::pwm::SetDutyCycle;
use syunit::*;

use crate::{Dismantle, Setup};

/// Default PWM-Frequency of an LED
pub const LED_PWM_FREQ : Velocity = Velocity(200.0);

/// An LED that can be dimmed
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
    /// Creates a new LED from a PWM-channel
    pub fn new(pwm : P) -> Self {
        Self {
            pwm,
            duty: 0
        }
    }

    // Read value
        /// Checks whether the LED is on
        pub fn is_on(&self) -> bool {
            self.duty == self.pwm.max_duty_cycle()
        }

        /// Checks whether the LED is off
        pub fn is_off(&self) -> bool {
            !self.is_on()
        }

        /// Returns the dim-factor of the LED
        pub fn dim_fac(&self) -> Factor {
            // Safe to use here
            unsafe { Factor::new_unchecked((self.duty as f32) / (self.pwm.max_duty_cycle() as f32)) }
        }
    // 

    // Write value
        /// Sets the LED to on
        pub fn set_on(&mut self) -> Result<(), P::Error> {
            self.pwm.set_duty_cycle_fully_on().map(|v| {
                self.duty = self.pwm.max_duty_cycle();
                v
            })
        }

        /// Sets the LED to off
        pub fn set_off(&mut self) -> Result<(), P::Error> {
            self.pwm.set_duty_cycle_fully_off().map(|v| {
                self.duty = 0;
                v
            })
        }

        /// Sets the led to a given boolean `val`
        pub fn set(&mut self, val : bool) -> Result<(), P::Error> {
            if val {
                self.set_on()
            } else {
                self.set_off()
            }
        }

        /// Dim the LED by a certain `factor`
        pub fn dim(&mut self, factor : Factor) -> Result<(), P::Error> {
            self.pwm.set_duty_cycle(factor.get_duty_for(self.pwm.max_duty_cycle()))
        }
    // 
}