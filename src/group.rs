use crate::{ActuatorError, SyncActuator, AdvancedActuator, SyncActuatorBlocking};
use crate::data::MicroSteps;
use crate::sync::StepperActuator;

use syunit::*;

// Implement macros
pub use syact_macros::*;

/// A group of components  
/// 
/// This trait allows a lot of functions to be used for all components at once.
/// 
/// # Generic type `T`
/// 
/// The generic type `T` repesents the components in the struct
pub trait ActuatorGroup<T, const C : usize>
where
    T : ?Sized + 'static
{
    // Iteration
        /// Execute a given function `func` for every element given by a readonly reference and it's index in the component group.
        /// 
        /// # Returns
        /// 
        /// Returns a fixed-sized array of the result type of the closure. The closure may not fail, see `try_for_each()` if a `Result` is required
        fn for_each<F, R>(&self, func : F) -> [R; C]
        where 
            F : FnMut(&T, usize) -> R;

        /// Execute a given function `func` for every element given by a mutable reference and it's index in the component group 
        /// 
        /// # Returns
        /// 
        /// Returns a fixed-sized array of the result type of the closure. The closure may not fail, see `try_for_each_mut()` if a `Result` is required
        fn for_each_mut<F, R>(&mut self, func : F) -> [R; C]
        where 
            F : FnMut(&mut T, usize) -> R;

        /// Execute a given function `func` for every element given by a mutable reference and it's index in the component group 
        /// 
        /// # Returns
        /// 
        /// Returns a fixed-sized array of the result type of the closure, wrapped into a `Result`, the closure has to return a `Result`
        /// 
        /// # Error
        /// 
        /// Stops executing if one of the components throws an error
        fn try_for_each<F, R, E>(&self, func : F) -> Result<[R; C], E>
        where 
            F : FnMut(&T, usize) -> Result<R, E>;

        /// Execute a given function `func` for every element given by a mutable reference and it's index in the component group 
        /// 
        /// # Returns
        /// 
        /// Returns a fixed-sized array of the result type of the closure, wrapped into a `Result`, the closure has to return a `Result`
        /// 
        /// # Error
        /// 
        /// Stops executing if one of the components throws an error
        fn try_for_each_mut<F, R, E>(&mut self, func : F) -> Result<[R; C], E>
        where 
            F : FnMut(&mut T, usize) -> Result<R, E>;
    //
}

/// A group of synchronous components  
/// 
/// This trait allows a lot of functions to be used for all components at once.
/// 
/// # Generic type `T`
/// 
/// The generic type `T` repesents the components in the struct
pub trait SyncActuatorGroup<T, const C : usize> : ActuatorGroup<T, C>
where 
    T : SyncActuator + ?Sized + 'static
{
    // Position
        /// Runs [SyncActuator::abs_pos()] for all components
        #[inline(always)]
        fn abs_pos(&self) -> [AbsPos; C] {
            self.for_each(|act, _| {
                act.abs_pos()
            })
        }
        
        /// Runs [SyncActuator::overwrite_abs_pos()] for all components
        #[inline(always)]
        fn overwrite_abs_pos(&mut self, abs_poss : &[AbsPos; C]) {
            self.for_each_mut(|act, index| {
                act.overwrite_abs_pos(abs_poss[index])
            });
        }

        /// Runs [SyncActuator::resolve_pos_limits_for_abs_pos()] for all components 
        #[inline(always)]
        fn limits_for_abs_pos(&self, abs_poss : &[AbsPos; C]) -> [RelDist; C] {
            self.for_each(|act, index| {
                act.resolve_pos_limits_for_abs_pos(abs_poss[index])
            })
        }

        /// Checks if the given abs_poss are vaild, which means they are finite and in range of the components
        #[inline(always)]
        fn valid_abs_pos(&self, abs_poss : &[AbsPos; C]) -> bool {
            self.for_each(|act, index| {
                !act.resolve_pos_limits_for_abs_pos(abs_poss[index]).is_normal() & abs_poss[index].is_finite()
            }).iter().all(|v| *v)
        }

        /// Same as [SyncActuatorGroup::valid_abs_pos()], but it evaluates the check for each component and returns seperated results for analysis
        #[inline(always)]
        fn valid_abs_pos_verbose(&self, abs_poss : &[AbsPos; C]) -> [bool; C] {
            self.for_each(|act, index| {
                !act.resolve_pos_limits_for_abs_pos(abs_poss[index]).is_normal() & abs_poss[index].is_finite()
            })
        }

        /// Runs [SyncActuator::set_endpos()] for all components
        #[inline(always)]
        fn set_endpos(&mut self, set_dist : &[AbsPos; C]) {
            self.for_each_mut(|act, index| {
                act.set_endpos(set_dist[index]);
            });
        }
        
        /// Runs [SyncActuator::set_pos_limits()] for all components
        #[inline(always)]
        fn set_pos_limits(&mut self, min : &[Option<AbsPos>; C], max : &[Option<AbsPos>; C]) {
            self.for_each_mut(|act, index| {
                act.set_pos_limits(min[index], max[index]);
            }); 
        }
    //

    // Velocity
        /// Returns the maximum velocity for each component of the group
        fn velocity_max(&self) -> [Option<Velocity>; C] {
            self.for_each(|act, _| {
                act.velocity_max()
            })
        }

        /// Set the maximum velocity of the components
        fn set_velocity_max(&mut self, velocity_opt : [Option<Velocity>; C]) -> Result<[(); C], ActuatorError> {
            self.try_for_each_mut(|act, index| {
                act.set_velocity_max(velocity_opt[index])
            })
        }
    // 

    // Acceleration
        /// Returns the maximum acceleration for each actuator of the group
        fn acceleration_max(&self) -> [Option<Acceleration>; C] {
            self.for_each(|act, _| {
                act.acceleration_max()
            })
        }

        /// Set the maximum acceleration for each actuator of the group
        fn set_acceleration_max(&mut self, acceleration_opt : [Option<Acceleration>; C]) -> Result<[(); C], ActuatorError> {
            self.try_for_each_mut(|act, index| {
                act.set_acceleration_max(acceleration_opt[index])
            })
        }
    //
}

// Automatic implementation
impl<G, T, const C : usize> SyncActuatorGroup<T, C> for G
where 
    G : ActuatorGroup<T, C>,
    T : SyncActuator + ?Sized + 'static
{ 
    // No definitions required
}

// ##############################################
// #    SyncActuatorGroup - Extention traits    #
// ##############################################
    // Movements
        /// Further extending a `SyncActuatorGroup` with blocking movements
        pub trait SyncActuatorBlockingGroup<T, const C : usize> : SyncActuatorGroup<T, C>
        where 
            T : SyncActuatorBlocking + ?Sized + 'static
        {
            /// Runs [SyncActuatorBlocking::drive_rel_blocking()] for all components
            fn drive_rel_blocking(&mut self, rel_dists : [RelDist; C], speed : [Factor; C]) -> Result<[(); C], ActuatorError> {
                self.try_for_each_mut(|act, index| {
                    act.drive_rel_blocking(rel_dists[index], speed[index])  
                })
            }

            /// Runs [SyncActuatorBlocking::drive_abs_blocking()] for all components
            fn drive_abs_blocking(&mut self, abs_pos : [AbsPos; C], speed : [Factor; C]) -> Result<[(); C], ActuatorError> {
                self.try_for_each_mut(|act, index| {
                    act.drive_abs_blocking(abs_pos[index], speed[index])
                })
            }
        }

        // Automatic implementation
        impl<G, T, const C : usize> SyncActuatorBlockingGroup<T, C> for G
        where 
            G : SyncActuatorGroup<T, C>,
            T : SyncActuatorBlocking + ?Sized + 'static
        { 
            // No definitions required
        }
    //
//

// ###############################
// #    AdvancedActuatorGroup    #
// ###############################
    /// Further extending `ActuatorGroup` with load application
    pub trait AdvancedActuatorGroup<T, const C : usize> : ActuatorGroup<T, C>
        where 
            T : AdvancedActuator + ?Sized + 'static
    {
        /// Apply an inertia to each of the components
        #[inline]
        fn apply_inertias(&mut self, inertias : &[Inertia; C]) -> Result<[(); C], ActuatorError> {
            self.try_for_each_mut(|act, index| {
                act.apply_inertia(inertias[index])
            })
        }

        /// Runs [SyncActuatorAdvanced::apply_gen_force()] for all components
        #[inline]
        fn apply_forces(&mut self, forces : &[Force; C]) -> Result<(), ActuatorError> {
            self.try_for_each_mut(|act, index| {
                act.apply_gen_force(forces[index])
            })?;
            Ok(())
        }
    }

    // Automatic implementation
    impl<G, T, const C : usize> AdvancedActuatorGroup<T, C> for G 
    where 
        G : ActuatorGroup<T, C>,
        T : AdvancedActuator + ?Sized + 'static
    { 
        // No definitions required
    }
// 

// StepperActuatorGroup
    /// A group of stepper motor based components
    pub trait StepperActuatorGroup<T, const C : usize> : SyncActuatorGroup<T, C> 
    where 
        T: StepperActuator + ?Sized + 'static
    {
        /// Returns the amount of microsteps every component uses
        fn microsteps(&self) -> [MicroSteps; C] {
            self.for_each(|comp, _| {
                comp.microsteps()
            })
        }

        /// Sets the amount of microsteps for each motor 
        fn set_micro(&mut self, micro : [MicroSteps; C]) -> Result<(), ActuatorError> {
            self.try_for_each_mut(|comp, index| {
                comp.set_microsteps(micro[index])
            })?;
            Ok(())
        }
    }

    impl<G, T, const C : usize> StepperActuatorGroup<T, C> for G 
    where 
        G : SyncActuatorGroup<T, C>,
        T : StepperActuator + ?Sized + 'static
    { 
        // No definitions required
    }
//