use crate::{StepperConst, SyncComp, SyncCompGroup};
use crate::math::{CurveBuilder, PathBuilder, PathNode};
use crate::units::*;

pub use syact_macros::StepperCompGroup;

/// A component based on a stepper motor
pub trait StepperComp : SyncComp {
    /// Returns the constants of the stepper motor
    fn consts(&self) -> &StepperConst;

    /// The amount of microsteps in a full step
    fn micro(&self) -> u8;

    /// Set the amount of microsteps in a full step
    fn set_micro(&mut self, micro : u8);

    /// The angle of a step
    fn step_ang(&self) -> Delta {
        self.consts().step_ang(self.micro())
    }

    /// Creates a new curve builder for the stepper motor
    fn create_curve_builder(&self, omega_0 : Omega) -> CurveBuilder {
        CurveBuilder::new(self.consts(), self.vars(), self.link(), omega_0, self.micro())
    }

    /// Drive from node to node (used for path algorithms, use as a normal drive function is not recommended)
    fn drive_nodes(&mut self, delta : Delta, omega_0 : Omega, omega_tar : Omega, corr : &mut (Delta, Time)) -> Result<(), crate::Error>;
}

impl<T : AsRef<dyn StepperComp> + AsMut<dyn StepperComp> + AsRef<dyn SyncComp> + AsMut<dyn SyncComp>> StepperComp for T {
    fn consts(&self) -> &StepperConst {
        <T as AsRef<dyn StepperComp>>::as_ref(self).consts()
    }

    fn micro(&self) -> u8 {
        <T as AsRef<dyn StepperComp>>::as_ref(self).micro()
    }

    fn set_micro(&mut self, micro : u8) {
        <T as AsMut<dyn StepperComp>>::as_mut(self).set_micro(micro)
    }

    fn drive_nodes(&mut self, delta : Delta, omega_0 : Omega, omega_tar : Omega, corr : &mut (Delta, Time)) -> Result<(), crate::Error> {
        <T as AsMut<dyn StepperComp>>::as_mut(self).drive_nodes(delta, omega_0, omega_tar, corr)
    }
}

/// A group of stepper motor based components
pub trait StepperCompGroup<const C : usize> : SyncCompGroup<C> {
    /// Create a new pathbuilder for the given stepper component group
    fn create_path_builder(&self, omega_0 : [Omega; C]) -> PathBuilder<C>;

    /// Drive from node to node (used for path algorithms, use as a normal drive function is not recommended)
    fn drive_nodes(&mut self, nodes_0 : &[PathNode; C], omega_tar : [Omega; C], corr : &mut [(Delta, Time); C]) -> Result<(), crate::Error>;

    /// Drive from node to node (used for path algorithms, use as a normal drive function is not recommended)
    fn drive_node_to_node(&mut self, nodes_0 : &[PathNode; C], nodes_tar : &[PathNode; C], corr : &mut [(Delta, Time); C]) -> Result<(), crate::Error> {
        let mut omegas = [Omega::ZERO; C];

        for i in 0 .. C {
            omegas[i] = nodes_tar[i].omega_0;
        }

        self.drive_nodes(nodes_0, omegas, corr)
    } 
}

impl<T : StepperComp, const C : usize> StepperCompGroup<C> for [T; C] {
    fn create_path_builder(&self, omega_0 : [Omega; C]) -> PathBuilder<C> {
        let mut builders = Vec::new();
        for i in 0 .. C {
            builders[i] = self[i].create_curve_builder(omega_0[i]);
        }
        PathBuilder::new(builders.try_into().unwrap())
    }

    fn drive_nodes(&mut self, nodes_0 : &[PathNode; C], omega_tar : [Omega; C], corr : &mut [(Delta, Time); C]) -> Result<(), crate::Error> {
        for i in 0 .. C {
            self[i].drive_nodes(nodes_0[i].delta, nodes_0[i].omega_0, omega_tar[i], &mut corr[i])?;
        }

        Ok(())
    }
}