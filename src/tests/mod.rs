use crate::prelude::BuilderError;
use crate::{Stepper, StepperConst};
use crate::act::stepper::GenericPWM;

mod act;
mod device;
mod data;
mod math;

// Helper structs
pub struct SimPin {
    pub pin : u8,
    pub state : bool,
    pub state_changes : u64
}

impl SimPin {
    fn new(pin : u8) -> Self {
        SimPin {
            pin,
            state: false,
            state_changes: 0
        }
    }

    fn new_gen() -> Self {
        SimPin::new(0)
    }
}

impl embedded_hal::digital::ErrorType for SimPin {
    type Error = embedded_hal::digital::ErrorKind;
}

impl embedded_hal::digital::OutputPin for SimPin {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.state = false;
        self.state_changes += 1;
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.state = true;
        self.state_changes += 1;
        Ok(())
    }
}

impl GenericPWM<SimPin, SimPin> {
    /// Creates a new generic PWM signal with default testing values
    pub fn new_gen() -> Self {
        Self::new(SimPin::new_gen(), SimPin::new_gen())
    }
}

impl Stepper<SimPin, SimPin> {
    /// Creates a new generic stepper with both pins set to [ERR_PIN](super::pin::ERR_PIN) just for simulation and testing purposes
    #[inline]
    pub fn new_gen() -> Result<Self, BuilderError> {
        Self::new(GenericPWM::new_gen(), StepperConst::GEN)
    }
}