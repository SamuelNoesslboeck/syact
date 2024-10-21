use core::ops::{Div, Mul};

use alloc::boxed::Box;
use alloc::sync::Arc;

use syunit::*;

use crate::{AsyncActuator, SyncActuator, SyncActuatorBlocking, ActuatorError, Interruptible, AdvancedActuator, SyncActuatorState, DefinedActuator};
use crate::data::MicroSteps;
use crate::sync::stepper::StepperActuator;

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
    pub trait RatioActuatorParent : ActuatorParent 
    where
        <Self::Input as UnitSet>::Time : From<<Self::Output as UnitSet>::Time>,

        <Self::Input as UnitSet>::Position : Div<Self::Ratio, Output = <Self::Output as UnitSet>::Position>,
        <Self::Input as UnitSet>::Distance : Div<Self::Ratio, Output = <Self::Output as UnitSet>::Distance>,
        <Self::Input as UnitSet>::Velocity : Div<Self::Ratio, Output = <Self::Output as UnitSet>::Velocity>,
        <Self::Input as UnitSet>::Acceleration : Div<Self::Ratio, Output = <Self::Output as UnitSet>::Acceleration>,
        <Self::Input as UnitSet>::Jolt : Div<Self::Ratio, Output = <Self::Output as UnitSet>::Jolt>,
        <Self::Input as UnitSet>::Force : Mul<Self::Ratio, Output = <Self::Output as UnitSet>::Force>,

        <Self::Output as UnitSet>::Position : Mul<Self::Ratio, Output = <Self::Input as UnitSet>::Position>,
        <Self::Output as UnitSet>::Distance : Mul<Self::Ratio, Output = <Self::Input as UnitSet>::Distance>,
        <Self::Output as UnitSet>::Velocity : Mul<Self::Ratio, Output = <Self::Input as UnitSet>::Velocity>,
        <Self::Output as UnitSet>::Acceleration : Mul<Self::Ratio, Output = <Self::Input as UnitSet>::Acceleration>,
        <Self::Output as UnitSet>::Jolt : Mul<Self::Ratio, Output = <Self::Input as UnitSet>::Jolt>,
        <Self::Output as UnitSet>::Force : Div<Self::Ratio, Output = <Self::Input as UnitSet>::Force>
    {
        type Input : UnitSet;
        type Output : UnitSet;
        type Ratio : From<f32> + Into<f32>;

        /// The linear ratio that defines the relation between the child and the parent component
        /// 
        /// # Ratio
        /// 
        /// For each radian/mm the child moves, the parent moves this distances *times the `ratio`*
        fn ratio(&self) -> Self::Ratio;

        // Automatic implementations
            #[inline]
            fn pos_for_child(&self, parent_abs_pos : <Self::Input as UnitSet>::Position) -> <Self::Output as UnitSet>::Position {
                parent_abs_pos / self.ratio()
            }

            #[inline]
            fn pos_for_parent(&self, child_abs_pos : <Self::Output as UnitSet>::Position) -> <Self::Input as UnitSet>::Position {
                child_abs_pos * self.ratio()
            }

            #[inline]
            fn rel_dist_for_chlid(&self, parent_rel_dist : <Self::Input as UnitSet>::Distance) -> <Self::Output as UnitSet>::Distance {
                parent_rel_dist / self.ratio()
            }

            #[inline]
            fn rel_dist_for_parent(&self, child_rel_dist : <Self::Output as UnitSet>::Distance) -> <Self::Input as UnitSet>::Distance {
                child_rel_dist * self.ratio()
            }

            // Velocity
                #[inline]
                fn velocity_for_child(&self, parent_velocity : <Self::Input as UnitSet>::Velocity) -> <Self::Output as UnitSet>::Velocity {
                    parent_velocity / self.ratio()
                }

                #[inline]
                fn velocity_for_parent(&self, child_velocity : <Self::Output as UnitSet>::Velocity) -> <Self::Input as UnitSet>::Velocity {
                    child_velocity * self.ratio()
                }
            // 
            
            // Acceleration
                #[inline]
                fn acceleration_for_child(&self, parent_alpha : <Self::Input as UnitSet>::Acceleration) -> <Self::Output as UnitSet>::Acceleration {
                    parent_alpha / self.ratio()
                }

                #[inline]
                fn acceleration_for_parent(&self, child_alpha : <Self::Output as UnitSet>::Acceleration) -> <Self::Input as UnitSet>::Acceleration {
                    child_alpha * self.ratio()
                }
            //
            
            // Jolt    
                #[inline]
                fn jolt_for_child(&self, parent_jolt : <Self::Input as UnitSet>::Jolt) -> <Self::Output as UnitSet>::Jolt {
                    parent_jolt / self.ratio()
                }

                #[inline]
                fn jolt_for_parent(&self, child_jolt : <Self::Output as UnitSet>::Jolt) -> <Self::Input as UnitSet>::Jolt {
                    child_jolt * self.ratio()
                }
            //

            // Implementation for Newtonmeter to Newtonmeter conversion
            #[inline]
            fn force_for_child(&self, parent_force : <Self::Input as UnitSet>::Force) -> <Self::Output as UnitSet>::Force {
                parent_force * self.ratio()
            }

            #[inline]
            fn force_for_parent(&self, child_force : <Self::Output as UnitSet>::Force) -> <Self::Input as UnitSet>::Force {
                child_force / self.ratio()
            }

            // Implementation for Newtonmeter to Newtonmeter conversion
            #[inline]
            fn inertia_for_child(&self, parent_inertia : <Self::Input as UnitSet>::Inertia) -> <Self::Output as UnitSet>::Inertia {
                // Small workaround
                <Self::Output as UnitSet>::Inertia::new(parent_inertia.into() * self.ratio().into() * self.ratio().into())
            }

            #[inline]
            fn inertia_for_parent(&self, child_intertia : <Self::Output as UnitSet>::Inertia) -> <Self::Input as UnitSet>::Inertia {
                // Small workaround
                <Self::Input as UnitSet>::Inertia::new(child_intertia.into() / self.ratio().into() / self.ratio().into())
            }
        // 

        // Error types
            fn error_for_parent(&self, error : ActuatorError<Self::Output>) -> ActuatorError<Self::Input> {
                match error {
                    ActuatorError::InvaldRelativeDistance(_) => todo!(),
                    ActuatorError::InvalidVelocity(_) => todo!(),
                    ActuatorError::VelocityTooHigh(_, _) => todo!(),
                    ActuatorError::InvalidAcceleration(_) => todo!(),
                    ActuatorError::InvalidJolt(_) => todo!(),
                    ActuatorError::InvalidTime(time) => ActuatorError::InvalidTime(<Self::Input as UnitSet>::Time::from(time)),

                    ActuatorError::IOError => ActuatorError::IOError,

                    ActuatorError::Overload => ActuatorError::Overload
                }
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
        impl<T : RatioActuatorParent> SyncActuator<T::Input> for T 
        where
            <T::Input as UnitSet>::Time : From<<T::Output as UnitSet>::Time>,

            <T::Input as UnitSet>::Position : Div<T::Ratio, Output = <T::Output as UnitSet>::Position>,
            <T::Input as UnitSet>::Distance : Div<T::Ratio, Output = <T::Output as UnitSet>::Distance>,
            <T::Input as UnitSet>::Velocity : Div<T::Ratio, Output = <T::Output as UnitSet>::Velocity>,
            <T::Input as UnitSet>::Acceleration : Div<T::Ratio, Output = <T::Output as UnitSet>::Acceleration>,
            <T::Input as UnitSet>::Jolt : Div<T::Ratio, Output = <T::Output as UnitSet>::Jolt>,
            <T::Input as UnitSet>::Force : Mul<T::Ratio, Output = <T::Output as UnitSet>::Force>,

            <T::Output as UnitSet>::Position : Mul<T::Ratio, Output = <T::Input as UnitSet>::Position>,
            <T::Output as UnitSet>::Distance : Mul<T::Ratio, Output = <T::Input as UnitSet>::Distance>,
            <T::Output as UnitSet>::Velocity : Mul<T::Ratio, Output = <T::Input as UnitSet>::Velocity>,
            <T::Output as UnitSet>::Acceleration : Mul<T::Ratio, Output = <T::Input as UnitSet>::Acceleration>,
            <T::Output as UnitSet>::Jolt : Mul<T::Ratio, Output = <T::Input as UnitSet>::Jolt>,
            <T::Output as UnitSet>::Force : Div<T::Ratio, Output = <T::Input as UnitSet>::Force>,

            T::Child : SyncActuator<T::Output>
        {
            // Position
                #[inline]
                fn pos(&self) -> <T::Input as UnitSet>::Position {
                    self.pos_for_parent(self.child().pos())
                }

                fn overwrite_abs_pos(&mut self, mut abs_pos : <T::Input as UnitSet>::Position) {
                    let abs_pos = self.pos_for_child(abs_pos);
                    self.child_mut().overwrite_abs_pos(abs_pos)
                }
            //

            // Velocity
                #[inline]
                fn velocity_max(&self) -> Option<<T::Input as UnitSet>::Velocity> {
                    self.child().velocity_max().map(|velocity| self.velocity_for_parent(velocity))
                }

                fn set_velocity_max(&mut self, mut velocity_opt : Option<<T::Input as UnitSet>::Velocity>) -> Result<(), ActuatorError<T::Input>> {
                    let velocity_opt = velocity_opt.map(|velocity| self.velocity_for_child(velocity));
                    self.child_mut().set_velocity_max(velocity_opt)
                        .map_err(|err| self.error_for_parent(err))
                }
            // 

            // Acceleration
                #[inline]
                fn acceleration_max(&self) -> Option<<T::Input as UnitSet>::Acceleration> {
                    self.child().acceleration_max().map(|acceleration| self.acceleration_for_parent(acceleration))
                }
                
                fn set_acceleration_max(&mut self, mut acceleration_opt : Option<<T::Input as UnitSet>::Acceleration>) -> Result<(), ActuatorError<U>> {
                    acceleration_opt = acceleration_opt.map(|acceleration| self.acceleration_for_child(acceleration));
                    self.child_mut().set_acceleration_max(acceleration_opt)
                }
            // 

            // Jolt
                #[inline]
                fn jolt_max(&self) -> Option<U::Jolt> {
                    self.child().jolt_max().map(|jolt| self.jolt_for_parent(jolt))
                }

                fn set_jolt_max(&mut self, mut jolt_opt : Option<U::Jolt>) -> Result<(), ActuatorError> {
                    jolt_opt = jolt_opt.map(|jolt| self.jolt_for_child(jolt));
                    self.child_mut().set_jolt_max(jolt_opt)
                }
            // 

            // Positional limits
                fn limit_max(&self) -> Option<AbsPos> {
                    self.child().limit_max().map(|limit| self.pos_for_child(limit))
                }

                fn limit_min(&self) -> Option<AbsPos> {
                    self.child().limit_min().map(|limit| self.pos_for_child(limit))
                }

                fn resolve_pos_limits_for_abs_pos(&self, abs_pos : AbsPos) -> RelDist {
                    self.rel_dist_for_parent(self.child().resolve_pos_limits_for_abs_pos(
                        self.pos_for_child(abs_pos)
                    ))
                }

                fn set_pos_limits(&mut self, mut min : Option<AbsPos>, mut max : Option<AbsPos>) {
                    min = min.map(|g| self.pos_for_child(g));
                    max = max.map(|g| self.pos_for_child(g));
                    self.child_mut().set_pos_limits(min, max)
                }

                fn set_endpos(&mut self, mut overwrite_abs_pos : AbsPos) {
                    overwrite_abs_pos = self.pos_for_child(overwrite_abs_pos);
                    self.child_mut().set_endpos(overwrite_abs_pos)
                }

                fn overwrite_pos_limits(&mut self, mut min : Option<AbsPos>, mut max : Option<AbsPos>) {
                    min = min.map(|g| self.pos_for_child(g));
                    max = max.map(|g| self.pos_for_child(g));
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

            fn drive_factor(&mut self, speed : Factor, direction : Direction) -> Result<(), ActuatorError> {
                self.child_mut().drive_factor(speed, direction)
            }

            fn drive_speed(&mut self, mut speed : Velocity) -> Result<(), ActuatorError> {
                speed = self.velocity_for_child(speed);
                self.child_mut().drive_speed(speed)
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

        fn step_dist(&self) -> RelDist {
            self.rel_dist_for_parent(self.child().step_dist())
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