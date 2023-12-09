use crate::act::Stepper;
use crate::act::stepper::GenericPWM;
use crate::act::StepperActuator;
use crate::data::StepperConst;

use serde::{Serialize, Deserialize};

/// A helper struct to enable serde IO for the `HRStepper` with a GenericPWM-Signal
#[derive(Debug, Clone, Serialize, Deserialize)]
struct StepperDes {
    /// The stepper constants
    pub consts : StepperConst,
    /// The direction pin of the stepper motor
    pub pin_dir : u8,
    /// The step pin of the stepper motor
    pub pin_step : u8
}

// JSON Parsing
impl TryFrom<StepperDes> for Stepper {
    type Error = crate::Error;

    fn try_from(des : StepperDes) -> Result<Self, Self::Error> {
        Ok(Stepper::new(
            GenericPWM::new(des.pin_step, des.pin_dir)?, 
            des.consts
        ))
    }
}

impl Into<StepperDes> for &Stepper {
    fn into(self) -> StepperDes {
        let device = self._ctrl.lock().unwrap();

        StepperDes { 
            consts: self.consts().clone(), 
            pin_dir: device.pin_dir(),
            pin_step: device.pin_step()
        }
    }
}

// JSON I/O
impl Serialize for Stepper {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
        where
            S: serde::Serializer {

        let ser : StepperDes = self.into();
        ser.serialize(serializer)
    }
}

impl<'de, 'a> Deserialize<'de> for Stepper {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
        where
            D: serde::Deserializer<'de> {
        let raw = StepperDes::deserialize(deserializer)?;
        Ok(Stepper::try_from(raw).unwrap())         // TODO: Remove unwrap!
    }
}