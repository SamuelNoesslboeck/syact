use crate::{StepperConst, SyncComp, SyncCompGroup};
use crate::math::StepTimeBuilder;
use crate::units::*;

pub use syact_macros::StepperCompGroup;

/// All path and curve calculations are designed for the 
pub trait StepperMotor : StepperComp {
    // Calculation
        fn torque_at_speed(&self, omega : Omega) -> Force;

        fn alpha_at_speed(&self, omega : Omega) -> Result<Alpha, crate::Error>;

        /// The approximate travel time for a point-to-point movement
        fn approx_time_ptp(&self, delta : Delta, speed_f : f32, acc : usize) -> Result<Time, crate::Error>;

        fn alpha_av(&self, omega_0 : Omega, omega_tar : Omega, acc : usize) -> Result<Alpha, crate::Error>;
    //

    /// Creates a new curve builder for the stepper motor
    fn create_builder(&self, omega_0 : Omega, omega_max : Omega) -> StepTimeBuilder;
    /// Drive from node to node (used for path algorithms, use as a normal drive function is not recommended)
    fn drive_nodes(&mut self, delta : Delta, omega_0 : Omega, omega_tar : Omega, corr : &mut (Delta, Time)) -> Result<(), crate::Error>;
}

/// A component based on a stepper motor
pub trait StepperComp : SyncComp {
    // Super comp
        #[inline]
        fn super_stepper_comp(&self) -> Option<&dyn StepperComp> {
            None
        }

        #[inline]
        fn super_stepper_comp_mut(&mut self) -> Option<&mut dyn StepperComp> {
            None
        }

        fn motor(&self) -> &dyn StepperMotor;

        fn motor_mut(&mut self) -> &mut dyn StepperMotor;
    // 

    /// Returns the constants of the stepper motor
    #[inline]
    fn consts(&self) -> &StepperConst {
        self.super_stepper_comp()
            .expect("Provide a super component or an override for this function!")
            .consts()
    }

    // Microstepping
        /// The amount of microsteps in a full step
        #[inline]
        fn micro(&self) -> u8 {
            self.super_stepper_comp()
                .expect("Provide a super component or an override for this function!")
                .micro()
        }

        /// Set the amount of microsteps in a full step
        fn set_micro(&mut self, micro : u8) {
            self.super_stepper_comp_mut()
                .expect("Provide a super component or an override for this function!")
                .set_micro(micro)
        }
    //

    // Math
        /// The angular distance of a step considering microstepping
        fn step_ang(&self) -> Delta {
            self.super_stepper_comp()
                .expect("Provide a super component or an override for this function!")
                .step_ang()
        }
    // 
}

// Helper implementation
impl<T : AsRef<dyn StepperComp> + AsMut<dyn StepperComp> + AsRef<dyn SyncComp> + AsMut<dyn SyncComp>> StepperComp for T {
    // Super Component
        fn super_stepper_comp(&self) -> Option<&dyn StepperComp> {
            <T as AsRef<dyn StepperComp>>::as_ref(self).super_stepper_comp()
        }

        fn super_stepper_comp_mut(&mut self) -> Option<&mut dyn StepperComp> {
            <T as AsMut<dyn StepperComp>>::as_mut(self).super_stepper_comp_mut()
        }

        fn motor(&self) -> &dyn StepperMotor {
            <T as AsRef<dyn StepperComp>>::as_ref(self).motor()
        }

        fn motor_mut(&mut self) -> &mut dyn StepperMotor {
            <T as AsMut<dyn StepperComp>>::as_mut(self).motor_mut()
        }
    // 
}

/// A group of stepper motor based components
pub trait StepperCompGroup<T, const C : usize> : SyncCompGroup<T, C> 
where T: StepperComp + ?Sized + 'static
{
    // /// Create a new pathbuilder for the given stepper component group
    // fn create_path_builder(&self, omega_0 : [Omega; C]) -> PathBuilder<C> {
    //     let builders = self.for_each_dyn(|comp, index| {
    //         comp.create_builder(omega_0[index])
    //     }); 
    //     PathBuilder::new(builders.try_into().unwrap())
    // }

    // /// Drive from node to node (used for path algorithms, use as a normal drive function is not recommended)
    // fn drive_nodes(&mut self, nodes_0 : &[PathNode; C], omega_tar : [Omega; C], corr : &mut [(Delta, Time); C]) -> Result<(), crate::Error> {
    //     self.try_for_each_mut(|comp, index| {
    //         comp.drive_nodes(nodes_0[index].delta, nodes_0[index].omega_0, omega_tar[index], &mut corr[index])
    //     })?; 
    //     Ok(())
    // }

    // /// Drive from node to node (used for path algorithms, use as a normal drive function is not recommended)
    // fn drive_node_to_node(&mut self, nodes_0 : &[PathNode; C], nodes_tar : &[PathNode; C], corr : &mut [(Delta, Time); C]) -> Result<(), crate::Error> {
    //     let mut omegas = [Omega::ZERO; C];

    //     for i in 0 .. C {
    //         omegas[i] = nodes_tar[i].omega_0;
    //     }

    //     self.drive_nodes(nodes_0, omegas, corr)
    // } 

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



impl<T : StepperComp + 'static, const C : usize> StepperCompGroup<T, C> for [T; C] { }