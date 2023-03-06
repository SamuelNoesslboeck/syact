use serde::{Serialize, Deserialize};

use crate::Component;
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

impl crate::math::MathActor for Cylinder 
{
    fn accel_dyn(&self, omega : Omega, gamma : Gamma) -> Alpha {
        self.alpha_for_this(
            self.ctrl.accel_dyn(self.omega_for_super(omega, gamma), self.gamma_for_super(gamma)), self.gamma_for_super(gamma))
    }
}

impl crate::meas::SimpleMeas for Cylinder 
{
    fn init_meas(&mut self, pin_meas : u16) {
        self.ctrl.init_meas(pin_meas)
    }
}

impl Component for Cylinder 
{
    // Data
        fn consts(&self) -> crate::StepperConst {
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
        
        fn gamma_for_super(&self, this_gamma : Gamma) -> Gamma {
            this_gamma / self.rte_ratio
        }

        fn gamma_for_this(&self, super_gamma : Gamma) -> Gamma {
            super_gamma * self.rte_ratio
        }
    // 

    // Link
        fn link(&mut self, lk : crate::data::LinkedData) {
            self.ctrl.link(lk);    
        }
    //

    // JSON I/O
        fn to_json(&self) -> Result<serde_json::Value, serde_json::Error> {
            serde_json::to_value(self)
        }
    // 

    // Loads
        fn apply_load_force(&mut self, force : Force) {
            self.ctrl.apply_load_force(force * self.rte_ratio / 1000.0)
        }

        fn apply_load_inertia(&mut self, inertia : Inertia) {
            self.ctrl.apply_load_inertia(inertia * self.rte_ratio * self.rte_ratio / 1000_000.0)
        }
    //
}