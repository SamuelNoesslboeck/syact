use serde::{Serialize, Deserialize};

use crate::{Component, StepperCtrl, StepperConst};
use crate::math::MathActor;
use crate::meas::SimpleMeas;
use crate::units::*;

/// A bearing powered by a motor with a certain gear ratio
#[derive(Debug, Serialize, Deserialize)]
pub struct GearBearing 
{
    /// Steppercontrol for the motor of the bearing
    pub ctrl : StepperCtrl,
    
    /// Angle ration from motor to bearing (omega_b / omega_m)
    pub ratio : f32
}

impl GearBearing {
    pub fn new(ctrl : StepperCtrl, ratio : f32) -> Self {
        Self {
            ctrl,
            ratio
        }
    }
}

impl SimpleMeas for GearBearing 
{
    fn init_meas(&mut self, pin_meas : u8) {
        self.ctrl.init_meas(pin_meas);
    }
}

impl MathActor for GearBearing
{
    fn accel_dyn(&self, omega : Omega, gamma : Gamma) -> Alpha {
        self.alpha_for_this(self.ctrl.accel_dyn(self.omega_for_super(omega, gamma), self.gamma_for_super(gamma)), self.gamma_for_super(gamma))
    }
}

impl Component for GearBearing 
{
    // Data
        fn consts(&self) -> StepperConst {
            self.ctrl.consts()
        }
    // 

    // Super
        fn super_comp(&self) -> Option<&dyn Component> {
            Some(&self.ctrl)
        }

        fn super_comp_mut(&mut self) -> Option<&mut dyn Component> {
            Some(&mut self.ctrl)
        }  

        /// Returns the angle for the motor from a given bearing angle
        fn gamma_for_super(&self, this_gamma : Gamma) -> Gamma {
            this_gamma / self.ratio
        }   

        /// Returns the angle for the motor from a given bearing angle
        fn gamma_for_this(&self, super_gamma : Gamma) -> Gamma {
            super_gamma * self.ratio
        }
    //

    // Json I/O
        fn to_json(&self) -> Result<serde_json::Value, serde_json::Error> {
            serde_json::to_value(self)
        }
    //
}