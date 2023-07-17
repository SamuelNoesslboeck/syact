#[cfg(feature = "std")]
use serde::{Serialize, Deserialize};

use crate::{SyncComp, Setup, StepperConst};
use crate::comp::stepper::StepperComp;
use crate::ctrl::Stepper;
use crate::units::*;

/// A cylinder using a stepper motor to power itself
pub type StepperCylinder = Cylinder<Stepper>;

/// Cylinder component struct
#[derive(Debug, Serialize, Deserialize)]
pub struct Cylinder<C : SyncComp> {
    /// Data of the connected stepper motor
    pub ctrl : C,

    /// Distance traveled per rad (Unit mm)   \
    /// f_rte = pitch / (2pi)
    pub rte_ratio : f32
}

impl<C : SyncComp> Cylinder<C> {
    /// Create a new cylinder instance
    pub fn new(ctrl : C, rte_ratio : f32) -> Self {
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
        fn write_data(&mut self, data : crate::data::CompData) {
            self.ctrl.write_data(data);    
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

impl<C : StepperComp> StepperComp for Cylinder<C> {
    fn consts(&self) -> &StepperConst {
        self.ctrl.consts()
    }

    fn micro(&self) -> u8 {
        self.ctrl.micro()
    }

    fn set_micro(&mut self, micro : u8) {
        self.ctrl.set_micro(micro) 
    }

    fn drive_nodes(&mut self, delta : Delta, omega_0 : Omega, omega_tar : Omega, corr : &mut (Delta, Time)) -> Result<(), crate::Error> {
        self.ctrl.drive_nodes(delta, omega_0, omega_tar, corr)
    }
}