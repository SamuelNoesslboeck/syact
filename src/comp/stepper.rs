use crate::{StepperConst, SyncComp, SyncCompGroup};
use crate::math::{CurveBuilder, PathBuilder};
use crate::units::*;

pub use stepper_macros::StepperCompGroup;

/// A component based on a stepper motor
pub trait StepperComp : SyncComp {
    /// Returns the constants of the stepper motor
    fn consts(&self) -> &StepperConst;

    /// Creates a new curve builder for the stepper motor
    fn create_curve_builder(&self, omega_0 : Omega) -> CurveBuilder {
        CurveBuilder::new(self.consts(), self.vars(), self.link(), omega_0)
    }
}

impl<T : AsRef<dyn StepperComp> + AsRef<dyn SyncComp> + AsMut<dyn SyncComp>> StepperComp for T {
    fn consts(&self) -> &StepperConst {
        <T as AsRef<dyn StepperComp>>::as_ref(self).consts()
    }
}

/// A group of stepper motor based components
pub trait StepperCompGroup<const C : usize> : SyncCompGroup<C> {
    /// Create a new pathbuilder for the given stepper component group
    fn create_path_builder(&self, omega_0 : [Omega; C]) -> PathBuilder<C>;
}

impl<T : StepperComp, const C : usize> StepperCompGroup<C> for [T; C] {
    fn create_path_builder(&self, omega_0 : [Omega; C]) -> PathBuilder<C> {
        let mut builders = Vec::new();
        for i in 0 .. C {
            builders[i] = self[i].create_curve_builder(omega_0[i]);
        }
        PathBuilder::new(builders.try_into().unwrap())
    }
}