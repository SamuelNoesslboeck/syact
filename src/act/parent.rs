use crate::data::MicroSteps;
use crate::math::movements::DefinedActuator;
use crate::prelude::StepperActuator;
use crate::{SyncActuator, Setup, StepperConfig, AsyncActuator};

use syunit::*;

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

            fn velocity_for_child(&self, parent_velocity : Velocity) -> Velocity {
                parent_velocity / self.ratio()
            }

            fn velocity_for_parent(&self, child_velocity : Velocity) -> Velocity {
                child_velocity * self.ratio()
            }

            fn alpha_for_child(&self, parent_alpha : Acceleration) -> Acceleration {
                parent_alpha / self.ratio()
            }

            fn alpha_for_parent(&self, child_alpha : Acceleration) -> Acceleration {
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
    impl<T : RatioActuatorParent + Setup> SyncActuator for T 
    where
        T::Child : SyncActuator
    {
        // Drive
            fn drive_rel(&mut self, mut delta : Delta, speed : Factor) -> Result<(), crate::Error> {
                delta = self.delta_for_chlid(delta);
                self.child_mut().drive_rel(delta, speed)
            }
        //  

        // Async
            /// Moves the component by the relative distance as fast as possible
            fn drive_rel_async(&mut self, mut delta : Delta, speed : Factor) -> Result<(), crate::Error> {
                delta = self.delta_for_chlid(delta);
                self.child_mut().drive_rel_async(delta, speed)
            }

            fn drive_velocity(&mut self, mut velocity_tar : Velocity) -> Result<(), crate::Error> {
                velocity_tar = self.velocity_for_child(velocity_tar);
                self.child_mut().drive_velocity(velocity_tar)
            }   

            fn await_inactive(&mut self) -> Result<(), crate::Error> {
                self.child_mut().await_inactive()
            }
        // 

        // Position
            fn gamma(&self) -> Gamma {
                self.gamma_for_parent(self.child().gamma())
            }

            fn set_gamma(&mut self, mut gamma : Gamma) {
                gamma = self.gamma_for_child(gamma);
                self.child_mut().set_gamma(gamma)
            }

            fn velocity_max(&self) -> Velocity {
                self.velocity_for_parent(self.child().velocity_max())
            }

            fn set_velocity_max(&mut self, mut velocity_max : Velocity) {
                velocity_max = self.velocity_for_child(velocity_max);
                self.child_mut().set_velocity_max(velocity_max)
            }

            fn limits_for_gamma(&self, gamma : Gamma) -> Delta {
                self.delta_for_parent(self.child().limits_for_gamma(
                    self.gamma_for_child(gamma)
                ))
            }

            fn set_limits(&mut self, mut min : Option<Gamma>, mut max : Option<Gamma>) {
                min = min.map(|g| self.gamma_for_child(g));
                max = max.map(|g| self.gamma_for_child(g));
                self.child_mut().set_limits(min, max)
            }

            fn set_end(&mut self, mut set_gamma : Gamma) {
                set_gamma = self.gamma_for_child(set_gamma);
                self.child_mut().set_end(set_gamma)
            }

            fn overwrite_limits(&mut self, mut min : Option<Gamma>, mut max : Option<Gamma>) {
                min = min.map(|g| self.gamma_for_child(g));
                max = max.map(|g| self.gamma_for_child(g));
                self.child_mut().overwrite_limits(min, max)
            }
        // 

        // Loads
            fn force_gen(&self) -> Force {
                self.force_for_parent(self.child().force_gen())
            }

            fn force_dir(&self) -> Force {
                self.force_for_parent(self.child().force_dir())
            }

            fn apply_gen_force(&mut self, mut force : Force) -> Result<(), crate::Error> {
                force = self.force_for_child(force);
                self.child_mut().apply_gen_force(force)
            }

            fn apply_dir_force(&mut self, mut force : Force) -> Result<(), crate::Error> {
                force = self.force_for_child(force);
                self.child_mut().apply_dir_force(force)
            }

            fn inertia(&self) -> Inertia {
                self.inertia_for_parent(self.child().inertia())
            }

            fn apply_inertia(&mut self, mut inertia : Inertia) {
                inertia = self.inertia_for_child(inertia);
                self.child_mut().apply_inertia(inertia)
            }
        // 
    }

    impl<T : RatioActuatorParent + Setup> StepperActuator for T 
    where
        T::Child : StepperActuator
    {
        fn consts(&self) -> &crate::StepperConst {
            self.child().consts()
        }

        // Config
            fn config(&self) -> &crate::StepperConfig {
                self.child().config()
            }

            fn set_config(&mut self, config : StepperConfig) {
                self.child_mut().set_config(config)
            }
        //

        // Microsteps
            fn microsteps(&self) -> MicroSteps {
                self.child().microsteps()
            }

            fn set_microsteps(&mut self, micro : MicroSteps) {
                self.child_mut().set_microsteps(micro)
            }
        // 

        fn step_ang(&self) -> Delta {
            self.delta_for_parent(self.child().step_ang())
        }
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

    impl<T : ActuatorParent + Setup> AsyncActuator for T
    where 
        T::Child : AsyncActuator 
    {
        fn drive(&mut self, dir : Direction, speed : Factor) -> Result<(), crate::Error> {
            self.child_mut().drive(dir, speed)
        }

        fn dir(&self) -> Direction {
            self.child().dir()
        }

        fn speed(&self) -> Factor {
            self.child().speed()
        }
    }

    // Movements
    impl<T : ActuatorParent> DefinedActuator for T 
    where
        T::Child : DefinedActuator
    {
        fn ptp_time_for_distance(&self, gamma_0 : Gamma, gamma_t : Gamma) -> Time {
            self.child().ptp_time_for_distance(gamma_0, gamma_t)
        }
    }
// 