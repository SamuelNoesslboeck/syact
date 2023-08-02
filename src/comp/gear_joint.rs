use serde::{Serialize, Deserialize};

use crate::{SyncComp, Stepper, Setup};
use crate::comp::stepper::StepperComp;
use crate::units::*;

/// A gear joint using a stepper motor to power itself
pub type StepperGearJoint = GearJoint<Stepper>;

/// A bearing powered by a motor with a certain gear ratio
#[derive(Debug, Serialize, Deserialize)]
pub struct GearJoint<C : SyncComp> {
    /// Steppercontrol for the motor of the bearing
    pub ctrl : C,
    
    /// Angle ration from motor to bearing (omega_b / omega_m)
    pub ratio : f32
}

impl<C : SyncComp> GearJoint<C> {
    /// Creates a new `Gearbearing`
    pub fn new(ctrl : C, ratio : f32) -> Self {
        Self {
            ctrl,
            ratio
        }
    }
}

impl<C : SyncComp> Setup for GearJoint<C> {
    fn setup(&mut self) -> Result<(), crate::Error> {
        self.ctrl.setup() 
    }
}

impl<C : SyncComp> SyncComp for GearJoint<C> {
    // Data
        fn data<'a>(&'a self) -> &'a crate::data::CompData {
            self.ctrl.data()
        }

        fn vars<'a>(&'a self) -> &'a crate::data::CompVars {
            self.ctrl.vars()
        }
    // 

    // Super
        fn parent_comp(&self) -> Option<&dyn SyncComp> {
            Some(&self.ctrl)
        }

        fn parent_comp_mut(&mut self) -> Option<&mut dyn SyncComp> {
            Some(&mut self.ctrl)
        }  

        /// Returns the angle for the motor from a given bearing angle
        fn gamma_for_parent(&self, this_gamma : Gamma) -> Gamma {
            this_gamma / self.ratio
        }   

        /// Returns the angle for the motor from a given bearing angle
        fn gamma_for_this(&self, parent_gamma : Gamma) -> Gamma {
            parent_gamma * self.ratio
        }
    //
}

impl<C : StepperComp> StepperComp for GearJoint<C> {
    #[inline]
    fn parent_stepper_comp(&self) -> Option<&dyn StepperComp> {
        Some(&self.ctrl)
    }

    #[inline]
    fn parent_stepper_comp_mut(&mut self) -> Option<&mut dyn StepperComp> {
        Some(&mut self.ctrl)
    }

    #[inline]
    fn motor(&self) -> &dyn crate::prelude::StepperMotor {
        self.ctrl.motor()
    }

    #[inline]
    fn motor_mut(&mut self) -> &mut dyn crate::prelude::StepperMotor {
        self.ctrl.motor_mut()
    }
}