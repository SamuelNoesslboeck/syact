use crate::{ActuatorVars, SyncActuator};
use crate::units::*;

use super::Interruptible;

pub trait ActuatorParent {
    type Child;

    fn child(&self) -> &Self::Child;

    fn child_mut(&mut self) -> &mut Self::Child; 
}

// Relationships
    /// A parent that relates to its child through a constant `ratio`
    pub trait RatioActuatorParent : ActuatorParent {
        /// The linear ratio that defines the relation between the child and the parent component
        /// 
        /// # Ratio
        /// 
        /// For each radian/mm the child moves, the parent moves this distances *times the `ratio`*
        fn ratio(&self) -> f32;

        // Automatic implementations
            fn gamma_for_child(&self, parent_gamma : Gamma) -> Gamma {
                parent_gamma / self.ratio()
            }

            fn gamma_for_parent(&self, child_gamma : Gamma) -> Gamma {
                child_gamma * self.ratio()
            }

            fn delta_for_chlid(&self, parent_delta : Delta) -> Delta {
                parent_delta / self.ratio()
            }

            fn delta_for_parent(&self, child_delta : Delta) -> Delta {
                child_delta * self.ratio()
            }

            fn omega_for_child(&self, parent_omega : Omega) -> Omega {
                parent_omega / self.ratio()
            }

            fn omega_for_parent(&self, child_omega : Omega) -> Omega {
                child_omega * self.ratio()
            }

            fn alpha_for_child(&self, parent_alpha : Alpha) -> Alpha {
                parent_alpha / self.ratio()
            }

            fn alpha_for_parent(&self, child_alpha : Alpha) -> Alpha {
                child_alpha * self.ratio()
            }

            // Implementation for Newtonmeter to Newtonmeter conversion
            fn force_for_child(&self, parent_force : Force) -> Force {
                parent_force * self.ratio()
            }

            fn force_for_parent(&self, child_force : Force) -> Force {
                child_force / self.ratio()
            }

            // Implementation for Newtonmeter to Newtonmeter conversion
            fn inertia_for_child(&self, parent_inertia : Inertia) -> Inertia {
                parent_inertia * self.ratio() * self.ratio()
            }

            fn inertia_for_parent(&self, child_intertia : Inertia) -> Inertia {
                child_intertia / self.ratio() / self.ratio()
            }
        // 
    }
// 

// ##################################
// #    AUTOMATIC IMPLEMENTATION    #
// ##################################
// 
// Automatically implements `SyncActor` for every component
    impl<T : RatioActuatorParent + sylo::Enable> SyncActuator for T 
    where
        T::Child : SyncActuator
    {
        // Data
            #[inline]
            fn vars(&self) -> &ActuatorVars {
                self.child().vars()
            }
        // 

        // Drive
            fn drive_rel(&mut self, delta : crate::prelude::Delta, speed_f : f32) -> Result<crate::prelude::Delta, crate::Error> {
                if (1.0 < speed_f) | (0.0 >= speed_f) {
                    panic!("Invalid speed factor! {}", speed_f)
                }

                Ok(self.delta_for_parent(
                    self.child_mut().drive_rel(self.delta_for_chlid(delta), speed_f)?
                ))
            }
        //  

        // Async
            /// Moves the component by the relative distance as fast as possible
            fn drive_rel_async(&mut self, delta : Delta, speed_f : f32) -> Result<(), crate::Error> {
                if (1.0 < speed_f) | (0.0 >= speed_f) {
                    panic!("Invalid speed factor! {}", speed_f)
                }

                self.child_mut().drive_rel_async(self.delta_for_chlid(delta), speed_f)
            }

            fn drive_omega(&mut self, omega_tar : Omega) -> Result<(), crate::Error> {
                self.child_mut().drive_omega(
                    self.omega_for_child(omega_tar)
                )
            }   

            fn await_inactive(&mut self) -> Result<Delta, crate::Error> {
                Ok(self.delta_for_parent(self.child_mut().await_inactive()?))
            }
        // 

        // Position
            fn gamma(&self) -> Gamma {
                self.gamma_for_parent(self.child().gamma())
            }

            fn set_gamma(&mut self, gamma : Gamma) {
                self.child_mut().set_gamma(self.gamma_for_child(gamma))
            }

            fn omega_max(&self) -> Omega {
                self.omega_for_parent(self.child().omega_max())
            }

            fn set_omega_max(&mut self, omega_max : Omega) {
                self.child_mut().set_omega_max(self.omega_for_child(omega_max))
            }

            fn limits_for_gamma(&self, gamma : Gamma) -> Delta {
                self.child().limits_for_gamma(
                    self.gamma_for_child(gamma)
                )
            }

            fn set_limits(&mut self, min : Option<Gamma>, max : Option<Gamma>) {
                self.child_mut().set_limits(
                    min.map(|g| self.gamma_for_child(g)), 
                    max.map(|g| self.gamma_for_child(g))
                )
            }

            fn set_end(&mut self, set_gamma : Gamma) {
                self.child_mut().set_end(
                    self.gamma_for_child(set_gamma)
                )
            }

            fn overwrite_limits(&mut self, min : Option<Gamma>, max : Option<Gamma>) {
                self.child_mut().overwrite_limits(
                    min.map(|g| self.gamma_for_child(g)),
                    max.map(|g| self.gamma_for_child(g))
                )
            }
        // 

        // Loads
            fn gen_force(&self) -> Force {
                self.force_for_parent(self.child().gen_force())
            }

            fn dir_force(&self) -> Force {
                self.force_for_parent(self.child().dir_force())
            }

            fn apply_gen_force(&mut self, force : Force) -> Result<(), crate::Error> {
                self.child_mut().apply_gen_force(
                    self.force_for_child(force)
                )
            }

            fn apply_dir_force(&mut self, force : Force) -> Result<(), crate::Error> {
                self.child_mut().apply_dir_force(
                    self.force_for_child(force)
                )
            }

            fn inertia(&self) -> Inertia {
                self.inertia_for_parent(self.child().inertia())
            }

            fn apply_inertia(&mut self, inertia : Inertia) {
                self.child_mut().apply_inertia(
                    self.inertia_for_child(inertia)
                )
            }
        // 
    }

    impl<T : ActuatorParent> Interruptible for T 
    where
        T::Child : Interruptible
    {
        fn add_interruptor(&mut self, interruptor : Box<dyn super::Interruptor + Send>) {
            self.child_mut().add_interruptor(interruptor)
        }

        fn intr_reason(&self) -> Option<super::InterruptReason> {
            self.child().intr_reason()
        }
    }
// 