use alloc::boxed::Box;
use alloc::sync::Arc;

use syunit::*;

use crate::{AsyncActuator, SyncActuator, SyncActuatorBlocking};
use crate::act::{ActuatorError, Interruptible, AdvancedActuator, SyncActuatorState};
use crate::act::stepper::StepperActuator;
use crate::data::MicroSteps;
use crate::math::movements::DefinedActuator;

/// A trait that marks an actuator which acts as a parent for another actuator
pub trait ActuatorParent {
    /// The type of the child
    type Child;

    /// Returns a reference to the child
    fn child(&self) -> &Self::Child;

    /// Returns a mutable reference to the child
    fn child_mut(&mut self) -> &mut Self::Child; 
}

// Relationships
    /// A parent that relates to its child through a constant `ratio`
    #[allow(missing_docs)]
    pub trait RatioActuatorParent : ActuatorParent {
        /// The linear ratio that defines the relation between the child and the parent component
        /// 
        /// # Ratio
        /// 
        /// For each radian/mm the child moves, the parent moves this distances *times the `ratio`*
        fn ratio(&self) -> f32;

        // Automatic implementations
            #[inline]
            fn abs_pos_for_child(&self, parent_abs_pos : AbsPos) -> AbsPos {
                parent_abs_pos / self.ratio()
            }

            #[inline]
            fn abs_pos_for_parent(&self, child_abs_pos : AbsPos) -> AbsPos {
                child_abs_pos * self.ratio()
            }

            #[inline]
            fn rel_dist_for_chlid(&self, parent_rel_dist : RelDist) -> RelDist {
                parent_rel_dist / self.ratio()
            }

            #[inline]
            fn rel_dist_for_parent(&self, child_rel_dist : RelDist) -> RelDist {
                child_rel_dist * self.ratio()
            }

            // Velocity
                #[inline]
                fn velocity_for_child(&self, parent_velocity : Velocity) -> Velocity {
                    parent_velocity / self.ratio()
                }

                #[inline]
                fn velocity_for_parent(&self, child_velocity : Velocity) -> Velocity {
                    child_velocity * self.ratio()
                }
            // 
            
            // Acceleration
                #[inline]
                fn acceleration_for_child(&self, parent_alpha : Acceleration) -> Acceleration {
                    parent_alpha / self.ratio()
                }

                #[inline]
                fn acceleration_for_parent(&self, child_alpha : Acceleration) -> Acceleration {
                    child_alpha * self.ratio()
                }
            //
            
            // Jolt    
                #[inline]
                fn jolt_for_child(&self, parent_jolt : Jolt) -> Jolt {
                    parent_jolt / self.ratio()
                }

                #[inline]
                fn jolt_for_parent(&self, child_jolt : Jolt) -> Jolt {
                    child_jolt * self.ratio()
                }
            //

            // Implementation for Newtonmeter to Newtonmeter conversion
            #[inline]
            fn force_for_child(&self, parent_force : Force) -> Force {
                parent_force * self.ratio()
            }

            #[inline]
            fn force_for_parent(&self, child_force : Force) -> Force {
                child_force / self.ratio()
            }

            // Implementation for Newtonmeter to Newtonmeter conversion
            #[inline]
            fn inertia_for_child(&self, parent_inertia : Inertia) -> Inertia {
                parent_inertia * self.ratio() * self.ratio()
            }

            #[inline]
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
    // SyncActuator traits
        impl<T : RatioActuatorParent> SyncActuator for T 
        where
            T::Child : SyncActuator
        {
            // Position
                fn abs_pos(&self) -> AbsPos {
                    self.abs_pos_for_parent(self.child().abs_pos())
                }

                fn overwrite_abs_pos(&mut self, mut abs_pos : AbsPos) {
                    abs_pos = self.abs_pos_for_child(abs_pos);
                    self.child_mut().overwrite_abs_pos(abs_pos)
                }
            //

            // Velocity
                fn velocity_max(&self) -> Option<Velocity> {
                    self.child().velocity_max().map(|velocity| self.velocity_for_parent(velocity))
                }

                fn set_velocity_max(&mut self, mut velocity_opt : Option<Velocity>) -> Result<(), ActuatorError> {
                    velocity_opt = velocity_opt.map(|velocity| self.velocity_for_child(velocity));
                    self.child_mut().set_velocity_max(velocity_opt)
                }
            // 

            // Acceleration
                fn acceleration_max(&self) -> Option<Acceleration> {
                    self.child().acceleration_max().map(|acceleration| self.acceleration_for_parent(acceleration))
                }
                
                fn set_acceleration_max(&mut self, mut acceleration_opt : Option<Acceleration>) -> Result<(), ActuatorError> {
                    acceleration_opt = acceleration_opt.map(|acceleration| self.acceleration_for_child(acceleration));
                    self.child_mut().set_acceleration_max(acceleration_opt)
                }
            // 

            // Jolt
                fn jolt_max(&self) -> Option<Jolt> {
                    self.child().jolt_max().map(|jolt| self.jolt_for_parent(jolt))
                }

                fn set_jolt_max(&mut self, mut jolt_opt : Option<Jolt>) -> Result<(), ActuatorError> {
                    jolt_opt = jolt_opt.map(|jolt| self.jolt_for_child(jolt));
                    self.child_mut().set_jolt_max(jolt_opt)
                }
            // 

            // Positional limits
                fn limit_max(&self) -> Option<AbsPos> {
                    self.child().limit_max().map(|limit| self.abs_pos_for_child(limit))
                }

                fn limit_min(&self) -> Option<AbsPos> {
                    self.child().limit_min().map(|limit| self.abs_pos_for_child(limit))
                }

                fn resolve_pos_limits_for_abs_pos(&self, abs_pos : AbsPos) -> RelDist {
                    self.rel_dist_for_parent(self.child().resolve_pos_limits_for_abs_pos(
                        self.abs_pos_for_child(abs_pos)
                    ))
                }

                fn set_pos_limits(&mut self, mut min : Option<AbsPos>, mut max : Option<AbsPos>) {
                    min = min.map(|g| self.abs_pos_for_child(g));
                    max = max.map(|g| self.abs_pos_for_child(g));
                    self.child_mut().set_pos_limits(min, max)
                }

                fn set_endpos(&mut self, mut overwrite_abs_pos : AbsPos) {
                    overwrite_abs_pos = self.abs_pos_for_child(overwrite_abs_pos);
                    self.child_mut().set_endpos(overwrite_abs_pos)
                }

                fn overwrite_pos_limits(&mut self, mut min : Option<AbsPos>, mut max : Option<AbsPos>) {
                    min = min.map(|g| self.abs_pos_for_child(g));
                    max = max.map(|g| self.abs_pos_for_child(g));
                    self.child_mut().overwrite_pos_limits(min, max)
                }
            // 
        }
    
        impl<T : RatioActuatorParent> SyncActuatorBlocking for T 
        where
            T::Child : SyncActuatorBlocking
        {
            // State
                fn state(&self) -> &dyn super::SyncActuatorState {
                    self.child().state() 
                }

                fn clone_state(&self) -> Arc<dyn SyncActuatorState> {
                    self.child().clone_state()
                }
            //  

            fn drive_rel_blocking(&mut self, mut rel_dist : RelDist, speed : Factor) -> Result<(), ActuatorError> {
                rel_dist = self.rel_dist_for_chlid(rel_dist);
                self.child_mut().drive_rel_blocking(rel_dist, speed)
            }
        }

        impl<T : RatioActuatorParent> AdvancedActuator for T
        where 
            T::Child : AdvancedActuator
        {
            // Loads
                fn force_gen(&self) -> Force {
                    self.force_for_parent(self.child().force_gen())
                }

                fn force_dir(&self) -> Force {
                    self.force_for_parent(self.child().force_dir())
                }

                fn apply_gen_force(&mut self, mut force : Force) -> Result<(), ActuatorError> {
                    force = self.force_for_child(force);
                    self.child_mut().apply_gen_force(force)
                }

                fn apply_dir_force(&mut self, mut force : Force) -> Result<(), ActuatorError> {
                    force = self.force_for_child(force);
                    self.child_mut().apply_dir_force(force)
                }

                fn inertia(&self) -> Inertia {
                    self.inertia_for_parent(self.child().inertia())
                }

                fn apply_inertia(&mut self, mut inertia : Inertia) -> Result<(), ActuatorError> {
                    inertia = self.inertia_for_child(inertia);
                    self.child_mut().apply_inertia(inertia)
                }
            //
        }
    // 

    impl<T : RatioActuatorParent> StepperActuator for T
    where
        T::Child : StepperActuator
    {
        // Microsteps
            fn microsteps(&self) -> MicroSteps {
                self.child().microsteps()
            }

            fn set_microsteps(&mut self, micro : MicroSteps) -> Result<(), ActuatorError> {
                self.child_mut().set_microsteps(micro)
            }
        // 

        fn step_ang(&self) -> RelDist {
            self.rel_dist_for_parent(self.child().step_ang())
        }
    }

    impl<T : ActuatorParent> Interruptible for T 
    where
        T::Child : Interruptible
    {
        fn add_interruptor(&mut self, interruptor : Box<dyn super::Interruptor + Send>) {
            self.child_mut().add_interruptor(interruptor)
        }

        fn intr_reason(&mut self) -> Option<super::InterruptReason> {
            self.child_mut().intr_reason()
        }
    }

    impl<T : ActuatorParent> AsyncActuator for T
    where 
        T::Child : AsyncActuator
    {
        
        fn drive_factor(&mut self, speed : Factor, dir : Direction) -> Result<(), ActuatorError> {
            self.child_mut().drive_factor(speed, dir)
        }

        fn drive_speed(&mut self, speed : Velocity) -> Result<(), ActuatorError> {
            self.child_mut().drive_speed(speed)
        }
    }

    // Movements
    impl<T : ActuatorParent> DefinedActuator for T 
    where
        T::Child : DefinedActuator
    {
        fn ptp_time_for_distance(&self, abs_pos_0 : AbsPos, abs_pos_t : AbsPos) -> Time {
            self.child().ptp_time_for_distance(abs_pos_0, abs_pos_t)
        }
    }
// 