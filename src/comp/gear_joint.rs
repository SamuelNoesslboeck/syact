use serde::{Serialize, Deserialize};

use crate::{SyncComp, StepperCtrl, Setup, StepperConst};
use crate::comp::stepper::StepperComp;
use crate::units::*;

/// A bearing powered by a motor with a certain gear ratio
#[derive(Debug, Serialize, Deserialize)]
pub struct GearJoint {
    /// Steppercontrol for the motor of the bearing
    pub ctrl : StepperCtrl,
    
    /// Angle ration from motor to bearing (omega_b / omega_m)
    pub ratio : f32
}

impl GearJoint {
    /// Creates a new `Gearbearing`
    pub fn new(ctrl : StepperCtrl, ratio : f32) -> Self {
        Self {
            ctrl,
            ratio
        }
    }
}

// impl crate::math::MathActor for GearJoint
// {
//     fn accel_dyn(&self, omega : Omega, gamma : Gamma) -> Alpha {
//         self.alpha_for_this(self.ctrl.accel_dyn(self.omega_for_super(omega, gamma), self.gamma_for_super(gamma)), self.gamma_for_super(gamma))
//     }
// }

impl Setup for GearJoint {
    fn setup(&mut self) -> Result<(), crate::Error> {
        self.ctrl.setup() 
    }
}

impl SyncComp for GearJoint {
    // Data
        fn link<'a>(&'a self) -> &'a crate::data::LinkedData {
            self.ctrl.link()
        }

        fn vars<'a>(&'a self) -> &'a crate::data::CompVars {
            self.ctrl.vars()
        }
    // 

    // Super
        fn super_comp(&self) -> Option<&dyn SyncComp> {
            Some(&self.ctrl)
        }

        fn super_comp_mut(&mut self) -> Option<&mut dyn SyncComp> {
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
}

impl StepperComp for GearJoint {
    fn consts(&self) -> &StepperConst {
        self.ctrl.consts()   
    }
}