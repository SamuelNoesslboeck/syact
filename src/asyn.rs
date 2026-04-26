use embedded_hal::pwm::SetDutyCycle;
use syunit::*;

use crate::ActuatorError;

/// A component which is asynchronous because of its hardware properties, e.g. a simple DC-Motors
pub trait AsyncActuator<U : UnitSet> {
    /* Getters */
        /// The current speed of the actuator
        fn speed(&self) -> U::Velocity;

        /// The current factor of the actuator, representing a "power level" where `1.0` is the full capacity
        fn factor(&self) -> Factor;

        /// The current direction of the actuator
        fn dir(&self) -> Direction;
    /**/

    /// Starts the movement process of the component in the given direction with a given `speed` factor
    async fn drive_factor(&mut self, speed : Factor, direction : Direction) -> Result<(), ActuatorError<U>>; 

    /// Start the movement process of the component with the given velocity `speed`, positive values for `speed` mean CW movement
    async fn drive_speed(&mut self, speed : U::Velocity) -> Result<(), ActuatorError<U>>;

    /// Fully halts the actuator
    async fn stop(&mut self) -> Result<(), ActuatorError<U>>;
}

/// A generic PWM DC-Motor driver with two PWM pins, one forward, the other backward
pub struct PwmDcDriver<FW : SetDutyCycle, BW : SetDutyCycle> 
where
    FW::Error : Into<ActuatorError>,
    BW::Error : Into<ActuatorError>
{
    /// Pin that controlls the forward PWM signal for the DcMotor controller
    pub pin_fw : FW,
    /// Pin that controlls the backward PWM signal for the DcMotor controller
    pub pin_bw : BW,

    /// Current factor that is being driven
    __factor : Factor,
    /// Current direction that is being driven
    __dir : Direction
}

impl<FW : SetDutyCycle, BW : SetDutyCycle> PwmDcDriver<FW, BW> 
where
    FW::Error : Into<ActuatorError>,
    BW::Error : Into<ActuatorError>
{
    /// Creates a new DcDriver
    pub fn init(pin_fw : FW, pin_bw : BW) -> Self {
        Self {
            pin_fw, 
            pin_bw,

            __factor: Factor::MIN,
            __dir: Default::default()
        }
    }

    /* Getters */
        /// Current factor that is being driven, with `1.0` being full capacity and `0.0` off
        pub fn factor(&self) -> Factor {
            self.__factor
        }

        /// Current movement direction
        pub fn dir(&self) -> Direction {
            self.__dir
        }
    /**/

    /// 
    pub fn drive_factor(&mut self, speed : Factor, direction : Direction) -> Result<(), ActuatorError> {
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

    /// Halts and turns off the actuator
    pub fn stop(&mut self) -> Result<(), ActuatorError<Rotary>> {
        self.pin_fw.set_duty_cycle_fully_off()
            .map_err(|err| err.into())?;
        self.pin_bw.set_duty_cycle_fully_off()
            .map_err(|err| err.into())?;
        Ok(())
    }
}

// TODO: Create DCMotor structure