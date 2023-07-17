use crate::ctrl::Stepper;
use crate::ctrl::stepper::GenericPWM;
use crate::comp::StepperComp;
use crate::data::StepperConst;

use serde::{Serialize, Deserialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
struct StepperDes 
{
    pub consts : StepperConst,
    pub pin_dir : u8,
    pub pin_step : u8
}

// JSON Parsing
impl TryFrom<StepperDes> for Stepper {
    type Error = crate::Error;

    fn try_from(des : StepperDes) -> Result<Self, Self::Error> {
        Ok(Stepper::new(GenericPWM::new(des.pin_step, des.pin_dir)?, des.consts))
    }
}

impl Into<StepperDes> for &Stepper {
    fn into(mut self) -> StepperDes {
        self.use_ctrl(|ctrl| {
            StepperDes { 
                consts: self.consts().clone(), 
                pin_dir: ctrl.pin_dir(),
                pin_step: ctrl.pin_step()
            }
        })
    }
}

// JSON_
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