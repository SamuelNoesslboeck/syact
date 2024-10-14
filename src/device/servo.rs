use embedded_hal::pwm::SetDutyCycle;
use syunit::*;

use crate::data::servo::ServoConst;

/// A basic servo motor with absolute position being controlled by a PWM signal
/// 
/// # Setup
/// 
/// In order to function correctly the servo has to be set up first!
/// - No pins will be occupied until the setup function is called
#[derive(Debug)]
pub struct MiniServo<P : SetDutyCycle> {
    /// The absolute position of the servo motor
    abs_pos : AbsPos,
    /// The constants of the servo motor (depending on type)
    _consts : ServoConst,

    /// The PWM output signal
    pwm : P
}

impl<P : SetDutyCycle> MiniServo<P> {
    /// Creates a new servo driver with the given servo data `consts` connected to the `pin_pwm`
    /// 
    /// # Setup
    /// 
    /// Before any use of movement functions you must call `start()` or `setup()` in order to set up all the pins
    pub fn new(consts : ServoConst, pwm : P) -> Self {
        Self {
            abs_pos: consts.default_pos(),
            _consts: consts,

            pwm
        }
    }
 
    /// Returns a reference to the `ServoConst` of the driver
    pub fn consts(&self) -> &ServoConst {
        &self._consts
    }

    // Drop
        /// Start the PWM signal and moves the servo to it's default position
        pub fn start(&mut self) -> Result<(), P::Error> {
            self.drive_default_pos()
        }

        /// Stops the servo driver and the PWM signal, can be started again with `start()` if desired
        pub fn stop(&mut self) -> Result<(), P::Error> {
            self.pwm.set_duty_cycle_fully_off()
        }
    //  

    // Absolute position
        /// Get the *aboslute* angle of the servo 
        pub fn abs_pos(&self) -> AbsPos {
            self.abs_pos
        }

        /// Set the absolute angle of the servo. Causes the servo to drive to this angle
        /// 
        /// # Panics
        /// 
        /// Panics if the given 
        pub fn drive_abs(&mut self, abs_pos : AbsPos) -> Result<(), P::Error> {
            self.drive_factor_pos(Factor::try_new(abs_pos / self._consts.abs_pos_max)
                .expect("The given position is either invalid or out of range!"))
        }
    // 
    
    // Factor
        /// Get the duty-cycle-percent of the motor (values `0.0` to `1.0`)
        pub fn factor(&self) -> Factor {
            // Safe to use, as all operations altering the Factor are internal
            unsafe { Factor::new_unchecked(self.abs_pos / self._consts.abs_pos_max) }
        }

        /// Set the duty-cycle percent of the servo (value `0.0` to `1.0`). Causes the servo to adapt its position
        pub fn drive_factor_pos(&mut self, factor : Factor) -> Result<(), P::Error> {
            self.pwm.set_duty_cycle(factor.get_duty_for(self.pwm.max_duty_cycle()))
        }
    // 

    // Positions
        /// Moves the servo to its default position
        pub fn drive_default_pos(&mut self) -> Result<(), P::Error> {
            self.drive_factor_pos(Factor::HALF)
        }

        /// Moves the servo to its minimum endpoint
        pub fn drive_endpoint_min(&mut self) -> Result<(), P::Error> {
            self.drive_factor_pos(Factor::MIN)
        }

        /// Moves the servo to its maximum endpoint
        pub fn goto_endpoint_max(&mut self) -> Result<(), P::Error> {
            self.drive_factor_pos(Factor::MAX)
        }
    //
}