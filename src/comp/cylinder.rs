#[cfg(feature = "std")]
use serde::{Serialize, Deserialize};

use crate::{SyncComp, Setup};
use crate::ctrl::StepperCtrl;

use crate::units::*;

/// Cylinder component struct
#[derive(Debug, Serialize, Deserialize)]
pub struct Cylinder
{
    /// Data of the connected stepper motor
    pub ctrl : StepperCtrl,

    /// Distance traveled per rad (Unit mm)   \
    /// f_rte = pitch / (2pi)
    pub rte_ratio : f32
}

impl Cylinder
{
    /// Create a new cylinder instance
    pub fn new(ctrl : StepperCtrl, rte_ratio : f32) -> Self {
        return Cylinder {
            ctrl,
            rte_ratio
        };
    }
}

// impl crate::math::MathActor for Cylinder 
// {
//     fn accel_dyn(&self, omega : Omega, gamma : Gamma) -> Alpha {
//         self.alpha_for_this(
//             self.ctrl.accel_dyn(self.omega_for_super(omega, gamma), self.gamma_for_super(gamma)), self.gamma_for_super(gamma))
//     }
// }

impl Setup for Cylinder {
    fn setup(&mut self) -> Result<(), crate::Error> {
        self.ctrl.setup()
    }
}

impl SyncComp for Cylinder 
{
    // Data
        fn consts<'a>(&'a self) -> &'a crate::StepperConst {
            self.ctrl.consts()
        }
        
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
        
        fn gamma_for_super(&self, this_gamma : Gamma) -> Gamma {
            this_gamma / self.rte_ratio
        }

        fn gamma_for_this(&self, super_gamma : Gamma) -> Gamma {
            super_gamma * self.rte_ratio
        }
    // 

    // Link
        fn write_link(&mut self, lk : crate::data::LinkedData) {
            self.ctrl.write_link(lk);    
        }
    //

    // Loads
        fn apply_force(&mut self, force : Force) {
            self.ctrl.apply_force(force * self.rte_ratio / 1000.0)
        }

        fn apply_inertia(&mut self, inertia : Inertia) {
            self.ctrl.apply_inertia(inertia * self.rte_ratio * self.rte_ratio / 1000_000.0)
        }
    //
}