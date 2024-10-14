use crate::act::{SyncActuator, ActuatorError};
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
    fn drive_rel(&mut self, rel_dists : [RelDist; C], speed : [Factor; C]) -> Result<[(); C], ActuatorError> {
        self.try_for_each_mut(|act, index| {
            act.drive_rel(rel_dists[index], speed[index])  
        })
    }

    /// Runs [SyncComp::drive_abs()] for all components
    fn drive_abs(&mut self, abs_pos : [AbsPos; C], speed : [Factor; C]) -> Result<[(); C], ActuatorError> {
        self.try_for_each_mut(|act, index| {
            act.drive_abs(abs_pos[index], speed[index])
        })
    }

    // Position
        /// Runs [SyncComp::abs_pos()] for all components
        #[inline(always)]
        fn abs_poss(&self) -> [AbsPos; C] {
            self.for_each(|act, _| {
                act.abs_pos()
            })
        }
        
        /// Runs [SyncComp::set_abs_pos()] for all components
        #[inline(always)]
        fn set_abs_poss(&mut self, abs_poss : &[AbsPos; C]) {
            self.for_each_mut(|act, index| {
                act.set_abs_pos(abs_poss[index])
            });
        }

        /// Runs [SyncComp::resolve_pos_limits_for_abs_pos()] for all components 
        #[inline(always)]
        fn limits_for_abs_poss(&self, abs_poss : &[AbsPos; C]) -> [RelDist; C] {
            self.for_each(|act, index| {
                act.resolve_pos_limits_for_abs_pos(abs_poss[index])
            })
        }

        /// Checks if the given abs_poss are vaild, which means they are finite and in range of the components
        #[inline(always)]
        fn valid_abs_poss(&self, abs_poss : &[AbsPos; C]) -> bool {
            self.for_each(|act, index| {
                !act.resolve_pos_limits_for_abs_pos(abs_poss[index]).is_normal() & abs_poss[index].is_finite()
            }).iter().all(|v| *v)
        }

        /// Same as [SyncCompGroup::valid_abs_poss()], but it evaluates the check for each component and returns seperated results for analysis
        #[inline(always)]
        fn valid_abs_poss_verb(&self, abs_poss : &[AbsPos; C]) -> [bool; C] {
            self.for_each(|act, index| {
                !act.resolve_pos_limits_for_abs_pos(abs_poss[index]).is_normal() & abs_poss[index].is_finite()
            })
        }

        /// Runs [SyncComp::set_end()] for all components
        #[inline(always)]
        fn set_ends(&mut self, set_dist : &[AbsPos; C]) {
            self.for_each_mut(|act, index| {
                act.set_endpos(set_dist[index]);
            });
        }
        
        /// Runs [SyncComp::set_pos_limits()] for all components
        #[inline(always)]
        fn set_pos_limits(&mut self, min : &[Option<AbsPos>; C], max : &[Option<AbsPos>; C]) {
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