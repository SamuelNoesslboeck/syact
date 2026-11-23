use embedded_hal::pwm::SetDutyCycle;
use syunit::*;

use crate::data::servo::{MiniServoConfig, MiniServoData};

/// A basic servo motor with absolute position being controlled by a PWM signal
/// 
/// # Setup
/// 
/// In order to function correctly the servo has to be set up first!
/// - No pins will be occupied until the setup function is called
#[derive(Debug)]
pub struct MiniServo<P : SetDutyCycle> {
    /// The constants of the servo motor (depending on type)
    pub data : MiniServoData,
    pub config : MiniServoConfig,

    /// The absolute position of the servo motor
    _pos : PositionRad,

    /// The PWM output signal
    pin_pwm : P
}

impl<P : SetDutyCycle> MiniServo<P> {
    /// Creates a new servo driver with the given servo data `consts` connected to the `pin_pwm`
    pub fn new(data : MiniServoData, config : MiniServoConfig, pin_pwm : P) -> Result<Self, P::Error> {
        let mut s = Self {
            data,
            config,

            _pos: PositionRad::ZERO,
            pin_pwm
        };

        s.drive_endpoint_min()?;

        Ok(s)
    }

    // Factor
        /// Get the duty-cycle-percent of the motor (values `0.0` to `1.0`)
        pub fn pos_factor(&self) -> Factor {
            // Safe to use, as all operations altering the Factor are internal
            unsafe { Factor::new_unchecked(self._pos / self.data.position_max) }
        }

        /// Set the duty-cycle percent of the servo (value `0.0` to `1.0`). Causes the servo to adapt its position
        pub fn drive_factor_pos(&mut self, factor : Factor) -> Result<(), P::Error> {
            self._pos = self.data.position_max * factor;
            self.pin_pwm.set_duty_cycle(
                self.data.duty_for_factor(factor).get_duty_for(self.pin_pwm.max_duty_cycle())
            )
        }
    // 


    // Absolute position
        /// Get the *aboslute* angle of the servo 
        pub fn pos(&self) -> PositionRad {
            self._pos
        }

        /// Set the absolute angle of the servo. Causes the servo to drive to this angle
        /// 
        /// # Panics
        /// 
        /// Panics if the given 
        pub fn drive_abs(&mut self, pos : PositionRad) -> Result<(), P::Error> {
            self.drive_factor_pos(Factor::try_new(pos / self.data.position_max)
                .expect("The given position is either invalid or out of range!"))
        }
    // 

    // Positions
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