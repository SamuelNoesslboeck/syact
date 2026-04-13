use embedded_hal::pwm::SetDutyCycle;
use syunit::*;

use crate::ActuatorError;

/// A component which is asynchronous because of its hardware properties, e.g. a simple DC-Motors
pub trait AsyncActuator<U : UnitSet> {
    // Getters
    /// The current speed of the actuator
    fn speed(&self) -> U::Velocity;

    /// The current factor of the actuator
    fn factor(&self) -> Factor;

    /// The current direction of the actuator
    fn dir(&self) -> Direction;

    /// Starts the movement process of the component in the given direction with a given `speed` factor
    fn drive_factor(&mut self, speed : Factor, direction : Direction) -> Result<(), ActuatorError<U>>; 

    /// Start the movement process of the component with the given velocity `speed`, positive values for `speed` mean CW movement
    fn drive_speed(&mut self, speed : U::Velocity) -> Result<(), ActuatorError<U>>;

    /// Fully halts the actuator
    fn stop(&mut self) -> Result<(), ActuatorError<U>>;
}

/// A generic PWM DC-Motor driver with two PWM pins, one for forward, the other for backward
/// 
/// Note that for DC-Motors speed is hard to define properly, especially under load, so mostly use the `drive_factor` functions
pub struct PWMDcDriver<FW : SetDutyCycle, BW : SetDutyCycle> {
    pin_fw : FW,
    pin_bw : BW,

    pub max_speed : RadPerSecond,

    __factor : Factor,
    __dir : Direction
}

impl<FW : SetDutyCycle, BW : SetDutyCycle> PWMDcDriver<FW, BW> {
    /// Creates a new DcDriver
    pub fn init(pin_fw : FW, pin_bw : BW, max_speed : RadPerSecond) -> Self {
        Self {
            pin_fw, pin_bw,
            max_speed,

            __factor: Factor::MIN,
            __dir: Default::default()
        }
    }
}

impl<FW : SetDutyCycle, BW : SetDutyCycle> AsyncActuator<Rotary> for PWMDcDriver<FW, BW> 
where
    FW::Error : Into<ActuatorError>,
    BW::Error : Into<ActuatorError>
{
    fn speed(&self) -> RadPerSecond {
        self.max_speed * self.__factor
    }

    fn factor(&self) -> Factor {
        self.__factor
    }

    fn dir(&self) -> Direction {
        self.__dir
    }

    fn drive_factor(&mut self, speed : Factor, direction : Direction) -> Result<(), ActuatorError> {
        self.__factor = speed;
        self.__dir = direction;

        // Set one pin to duty, the other gets turned off
        if direction.as_bool() {    
            self.pin_fw.set_duty_cycle((speed.as_f32() * self.pin_fw.max_duty_cycle() as f32) as u16)
                .map_err(|err| err.into())?;
            self.pin_bw.set_duty_cycle_fully_off()
                .map_err(|err| err.into())?;
        } else {
            self.pin_fw.set_duty_cycle_fully_off()
                .map_err(|err| err.into())?;
            self.pin_bw.set_duty_cycle((speed.as_f32() * self.pin_fw.max_duty_cycle() as f32) as u16)
                .map_err(|err| err.into())?;
        }

        Ok(())
    }

    fn drive_speed(&mut self, speed : RadPerSecond) -> Result<(), ActuatorError> {
        let dir = Direction::from_bool(speed.is_sign_positive());
        let fac = Factor::new(speed.abs() / self.max_speed);
        self.drive_factor(fac, dir)
    }
    
    fn stop(&mut self) -> Result<(), ActuatorError<Rotary>> {
        self.pin_fw.set_duty_cycle_fully_off()
            .map_err(|err| err.into())?;
        self.pin_bw.set_duty_cycle_fully_off()
            .map_err(|err| err.into())?;
        Ok(())
    }
}
