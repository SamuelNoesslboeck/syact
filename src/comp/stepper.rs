use crate::{StepperConst, SyncComp, SyncCompGroup};
use crate::math::{CurveBuilder, PathBuilder, PathNode};
use crate::units::*;

pub use syact_macros::StepperCompGroup;

/// A component based on a stepper motor
pub trait StepperComp : SyncComp {
    /// Returns the constants of the stepper motor
    fn consts(&self) -> &StepperConst;

    // Microstepping
        /// The amount of microsteps in a full step
        fn micro(&self) -> u8;

        /// Set the amount of microsteps in a full step
        fn set_micro(&mut self, micro : u8);
    //

    /// The angle of a step
    fn step_ang(&self) -> Delta {
        self.consts().step_ang(self.micro())
    }

    /// Creates a new curve builder for the stepper motor
    fn create_curve_builder(&self, omega_0 : Omega) -> CurveBuilder {
        CurveBuilder::new(self.consts(), self.vars(), self.data(), omega_0, self.micro())
    }

    /// Drive from node to node (used for path algorithms, use as a normal drive function is not recommended)
    fn drive_nodes(&mut self, delta : Delta, omega_0 : Omega, omega_tar : Omega, corr : &mut (Delta, Time)) -> Result<(), crate::Error>;
}

// Helper implementation
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
pub trait StepperCompGroup<const C : usize> : SyncCompGroup<C> 
where 
    Self::Comp : StepperComp
{
    /// Create a new pathbuilder for the given stepper component group
    fn create_path_builder(&self, omega_0 : [Omega; C]) -> PathBuilder<C> {
        let builders = self.for_each_dyn(|comp, index| {
            comp.create_curve_builder(omega_0[index])
        }); 
        PathBuilder::new(builders.try_into().unwrap())
    }

    /// Drive from node to node (used for path algorithms, use as a normal drive function is not recommended)
    fn drive_nodes(&mut self, nodes_0 : &[PathNode; C], omega_tar : [Omega; C], corr : &mut [(Delta, Time); C]) -> Result<(), crate::Error> {
        self.try_for_each_mut(|comp, index| {
            comp.drive_nodes(nodes_0[index].delta, nodes_0[index].omega_0, omega_tar[index], &mut corr[index])
        })?; 
        Ok(())
    }

    /// Drive from node to node (used for path algorithms, use as a normal drive function is not recommended)
    fn drive_node_to_node(&mut self, nodes_0 : &[PathNode; C], nodes_tar : &[PathNode; C], corr : &mut [(Delta, Time); C]) -> Result<(), crate::Error> {
        let mut omegas = [Omega::ZERO; C];

        for i in 0 .. C {
            omegas[i] = nodes_tar[i].omega_0;
        }

        self.drive_nodes(nodes_0, omegas, corr)
    } 

    /// Returns the amount of microsteps every component uses
    fn micro(&self) -> [u8; C] {
        self.for_each(|comp, _| {
            comp.micro()
        })
    }

    /// Sets the amount of microsteps for each motor 
    fn set_micro(&mut self, micro : [u8; C]) {
        self.for_each_mut(|comp, index| {
            comp.set_micro(micro[index]);
        });
    }
}

impl<T : StepperComp, const C : usize> StepperCompGroup<C> for [T; C] { }