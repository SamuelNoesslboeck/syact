use serde::{Serialize, Deserialize};

use crate::{SyncComp, Setup};
use crate::comp::stepper::StepperComp;
use crate::ctrl::Stepper;
use crate::units::*;

/// A cylinder using a stepper motor to power itself
pub type StepperCylinder = Cylinder<Stepper>;

/// Cylinder component struct
/// 
/// # Cylinders
/// 
/// Cylinders are practical for converting rotational movement into linear movement, most of the time using spindles 
/// or straps
#[derive(Debug, Serialize, Deserialize)]
pub struct Cylinder<C : SyncComp> {
    /// The parent component driving the cylinder
    pub ctrl : C,

    /// Distance traveled per rad (Unit mm)   \
    /// `f_rte = pitch / (2pi)`
    pub rte_ratio : f32
}

impl<C : SyncComp> Cylinder<C> {
    /// Create a new cylinder instance
    /// - `ctrl`: The parent component driving the cylinder
    /// - `rte_ratio`: (radius to extension ratio), millimeters travelled per radian the spindle rotated
    ///   - `f_rte = pitch / (2pi)`
    pub fn new(ctrl : C, rte_ratio : f32) -> Self {
        return Cylinder {
            ctrl,
            rte_ratio
        };
    }
}

impl<C : SyncComp> Setup for Cylinder<C> {
    fn setup(&mut self) -> Result<(), crate::Error> {
        self.ctrl.setup()
    }
}

impl<C : SyncComp> SyncComp for Cylinder<C> {
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
        
        fn gamma_for_parent(&self, this_gamma : Gamma) -> Gamma {
            this_gamma / self.rte_ratio
        }

        fn gamma_for_this(&self, parent_gamma : Gamma) -> Gamma {
            parent_gamma * self.rte_ratio
        }
    // 

    // Link
        fn write_data(&mut self, data : crate::data::CompData) {
            self.ctrl.write_data(data);    
        }
    //

    // Loads
        fn apply_gen_force(&mut self, force : Force) -> Result<(), crate::Error> {
            self.ctrl.apply_gen_force(force * self.rte_ratio / 1000.0)
        }

        fn apply_inertia(&mut self, inertia : Inertia) {
            self.ctrl.apply_inertia(inertia * self.rte_ratio * self.rte_ratio / 1000_000.0)
        }
    //
}

impl<C : StepperComp> StepperComp for Cylinder<C> {
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