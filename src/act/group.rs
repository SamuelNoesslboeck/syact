use crate::act::{SyncActuator, SyncActuatorError};
use crate::prelude::BuilderError;

use syunit::*;

/// A group of synchronous components  
/// This trait allows a lot of functions to be used for all components at once.
/// 
/// # Generic type `T`
/// 
/// The generic type `T` repesents the components in the struct
/// 
/// # Constant generic `C`
pub trait SyncActuatorGroup<T, const C : usize>
where 
    T : SyncActuator + ?Sized + 'static
{
    // Iteration
        /// Execute a given function `func` for every element given by a readonly reference and it's index in the component group.
        /// 
        /// # Returns
        /// 
        /// Returns a fixed-sized array of the result type of the closure. The closure may not fail, see `try_for_each()` if a `Result` is required
        fn for_each<'a, F, R>(&'a self, func : F) -> [R; C]
        where 
            F : FnMut(&'a T, usize) -> R;

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
        fn try_for_each<'a, F, R, E>(&'a self, func : F) -> Result<[R; C], E>
        where 
            F : FnMut(&'a T, usize) -> Result<R, E>;

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

    /// Runs [SyncComp::drive_rel()] for all components
    fn drive_rel(&mut self, deltas : [Delta; C], speed : [Factor; C]) -> Result<[(); C], SyncActuatorError> {
        self.try_for_each_mut(|act, index| {
            act.drive_rel(deltas[index], speed[index])  
        })
    }

    /// Runs [SyncComp::drive_abs()] for all components
    fn drive_abs(&mut self, gamma : [Gamma; C], speed : [Factor; C]) -> Result<[(); C], SyncActuatorError> {
        self.try_for_each_mut(|act, index| {
            act.drive_abs(gamma[index], speed[index])
        })
    }

    // Position
        /// Runs [SyncComp::gamma()] for all components
        #[inline(always)]
        fn gammas(&self) -> [Gamma; C] {
            self.for_each(|act, _| {
                act.gamma()
            })
        }
        
        /// Runs [SyncComp::set_gamma()] for all components
        #[inline(always)]
        fn set_gammas(&mut self, gammas : &[Gamma; C]) {
            self.for_each_mut(|act, index| {
                act.set_gamma(gammas[index])
            });
        }

        /// Runs [SyncComp::resolve_pos_limits_for_gamma()] for all components 
        #[inline(always)]
        fn limits_for_gammas(&self, gammas : &[Gamma; C]) -> [Delta; C] {
            self.for_each(|act, index| {
                act.resolve_pos_limits_for_gamma(gammas[index])
            })
        }

        /// Checks if the given gammas are vaild, which means they are finite and in range of the components
        #[inline(always)]
        fn valid_gammas(&self, gammas : &[Gamma; C]) -> bool {
            self.for_each(|act, index| {
                !act.resolve_pos_limits_for_gamma(gammas[index]).is_normal() & gammas[index].is_finite()
            }).iter().all(|v| *v)
        }

        /// Same as [SyncCompGroup::valid_gammas()], but it evaluates the check for each component and returns seperated results for analysis
        #[inline(always)]
        fn valid_gammas_verb(&self, gammas : &[Gamma; C]) -> [bool; C] {
            self.for_each(|act, index| {
                !act.resolve_pos_limits_for_gamma(gammas[index]).is_normal() & gammas[index].is_finite()
            })
        }

        /// Runs [SyncComp::set_end()] for all components
        #[inline(always)]
        fn set_ends(&mut self, set_dist : &[Gamma; C]) {
            self.for_each_mut(|act, index| {
                act.set_endpos(set_dist[index]);
            });
        }
        
        /// Runs [SyncComp::set_pos_limits()] for all components
        #[inline(always)]
        fn set_pos_limits(&mut self, min : &[Option<Gamma>; C], max : &[Option<Gamma>; C]) {
            self.for_each_mut(|act, index| {
                act.set_pos_limits(min[index], max[index]);
            }); 
        }
    //

    // Load calculation
        /// Runs [SyncComp::apply_inertia()] for all components
        #[inline(always)]
        fn apply_inertias(&mut self, inertias : &[Inertia; C]) {
            self.for_each_mut(|act, index| {
                act.apply_inertia(inertias[index]);
            }); 
        }

        /// Runs [SyncComp::apply_force_gen()] for all components
        #[inline]
        fn apply_forces(&mut self, forces : &[Force; C]) -> Result<(), BuilderError> {
            self.try_for_each_mut(|act, index| {
                act.apply_gen_force(forces[index])
            })?;
            Ok(())
        }

        /// Returns the maximum velocitys for each component of the group
        fn velocity_max(&self) -> [Velocity; C] {
            self.for_each(|act, _| {
                act.velocity_max()
            })
        }

        /// Set the maximum velocity  of the components
        fn set_velocity_max(&mut self, velocity_max : [Velocity; C]) {
            self.for_each_mut(|act, index| {
                act.set_velocity_max(velocity_max[index]);
            });
        }
    // 
}