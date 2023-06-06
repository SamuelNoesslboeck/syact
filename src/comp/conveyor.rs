#[cfg(feature = "std")]
use serde::{Serialize, Deserialize}; 

use crate::{StepperCtrl, SyncComp, Setup, StepperConst, LinkedData, AsyncComp};
use crate::comp::CompVars;
use crate::units::*;

use super::stepper::StepperComp;

/// A simple conveyor powered by a stepper motor
#[derive(Debug)]
#[cfg_attr(feature = "std", derive(Serialize, Deserialize))]
pub struct Conveyor {
    ctrl : StepperCtrl,

    /// Radius of the powered conveyor roll
    pub r_roll : f32
}

impl Conveyor {
    /// Creates a new instance of a conveyor
    pub fn new(ctrl : StepperCtrl, r_roll : f32) -> Self {
        Self {
            ctrl, 
            r_roll
        }
    }
}

impl Setup for Conveyor {
    fn setup(&mut self) -> Result<(), crate::Error> {
        self.ctrl.setup()
    }
}

impl SyncComp for Conveyor {
    fn vars<'a>(&'a self) -> &'a CompVars {
        self.ctrl.vars()
    }

    fn link<'a>(&'a self) -> &'a LinkedData {
        self.ctrl.link() 
    }

    // Super component
        fn super_comp(&self) -> Option<&dyn SyncComp> {
            Some(&self.ctrl)
        }

        fn super_comp_mut(&mut self) -> Option<&mut dyn SyncComp> {
            Some(&mut self.ctrl)
        }
    // 

    // Conversion
        fn gamma_for_super(&self, this_gamma : Gamma) -> Gamma {
            this_gamma / self.r_roll
        }

        fn gamma_for_this(&self, super_gamma : Gamma) -> Gamma {
            super_gamma * self.r_roll
        }
    // 

    fn apply_force(&mut self, mut force : Force) {
        force = force * self.r_roll / 1000.0;   // Millimeters to 
        self.ctrl.apply_force(force)
    }

    fn apply_inertia(&mut self, mut inertia : Inertia) {
        inertia = inertia * self.r_roll * self.r_roll / 1_000_000.0;
        self.ctrl.apply_inertia(inertia)
    }
}

impl AsyncComp for Conveyor {
    fn drive(&mut self, dir : crate::Direction, speed_f : f32) -> Result<(), crate::Error> {
        self.ctrl.drive(dir, speed_f)
    }

    fn dir(&self) -> crate::Direction {
        self.ctrl.dir()
    }

    fn speed_f(&self) -> f32 {
        self.ctrl.speed_f()
    }
}

impl StepperComp for Conveyor {
    fn consts(&self) -> &StepperConst {
        self.ctrl.consts()
    }
}