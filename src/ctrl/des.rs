use crate::ctrl::Stepper;
use crate::data::StepperConst;

use serde::{Serialize, Deserialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
struct StepperCtrlDes 
{
    pub consts : StepperConst,
    pub pin_dir : u8,
    pub pin_step : u8
}

// JSON Parsing
impl From<StepperCtrlDes> for Stepper {
    fn from(des : StepperCtrlDes) -> Self {
        Stepper::new(des.consts, des.pin_dir, des.pin_step)
    }
}

impl Into<StepperCtrlDes> for Stepper {
    fn into(self) -> StepperCtrlDes {
        #[cfg(feature = "std")]
        let pins = self.sys.lock().unwrap(); 

        #[cfg(not(feature = "std"))]
        let pins = self.sys;

        StepperCtrlDes { 
            consts: self.consts, 
            pin_dir: pins.dir.pin,
            pin_step: pins.step.pin
        }
    }
}

// JSON_
impl Serialize for Stepper {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
        where
            S: serde::Serializer {
        #[cfg(feature = "std")]
        let pins = self.sys.lock().unwrap();

        #[cfg(not(feature = "std"))]
        let pins = &self.sys;

        let raw : StepperCtrlDes = StepperCtrlDes { 
            consts: self.consts.clone(), 
            pin_dir: pins.dir.pin,
            pin_step: pins.step.pin
        };
        raw.serialize(serializer)
    }
}

impl<'de, 'a> Deserialize<'de> for Stepper {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
        where
            D: serde::Deserializer<'de> {
        let raw = StepperCtrlDes::deserialize(deserializer)?;
        Ok(Stepper::from(raw))
    }
}