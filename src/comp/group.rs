use alloc::boxed::Box;
use alloc::vec::Vec;

use core::ops::Index;
use core::ops::IndexMut;

use crate::SyncComp;
use crate::units::*;

/// A group of synchronous components that can be implemented for any type of array, vector or list as long as it can be indexed.
/// This trait then allows a lot of functions to be used to execute functions for all components at once.
pub trait SyncCompGroup<T, const COMP : usize> : IndexMut<usize, Output = Box<T>> + Index<usize, Output = Box<T>>
    where
        T: SyncComp,
        T: ?Sized
{
    // Setup
        /// Runs [SyncComp::setup()] for all components in the group
        fn setup(&mut self) {
            for i in 0 .. COMP {
                self[i].setup();
            }
        }

        /// Runs [SyncComp::setup_async()] for all components in the group 
        /// 
        /// # Feature 
        ///
        /// This function is only available if the "std" feature is enabled
        #[cfg(feature = "std")]
        fn setup_async(&mut self) {
            for i in 0 .. COMP {
                self[i].setup_async();
            }
        }
    // 

    // Data
        /// Runs [SyncComp::write_link()] for all components in the group. Note that the function is using the same 
        /// [LinkedData](crate::data::LinkedData) for all components
        fn write_link(&mut self, lk : crate::data::LinkedData) {
            for i in 0 .. COMP {
                self[i].write_link(lk.clone())
            }
        }
    //

    /// Runs [SyncComp::drive_rel()] for all components
    fn drive_rel(&mut self, deltas : [Delta; COMP], omegas : [Omega; COMP]) -> Result<[Delta; COMP], crate::Error> {
        let mut res = [Delta::ZERO; COMP];
        for i in 0 .. COMP {
            res[i] = self[i].drive_rel(deltas[i], omegas[i])?;
        }
        Ok(res)
    }

    /// Runs [SyncComp::drive_abs()] for all components
    fn drive_abs(&mut self, gamma : [Gamma; COMP], omegas : [Omega; COMP]) -> Result<[Delta; COMP], crate::Error>  {
        let mut res = [Delta::ZERO; COMP];
        for i in 0 .. COMP {
            res[i] = self[i].drive_abs(gamma[i], omegas[i])?;
        }
        Ok(res)
    }

    /// Runs [SyncComp::measure()] for all components in the group
    fn measure(&mut self, deltas : [Delta; COMP], omegas : [Omega; COMP], set_dist : [Gamma; COMP]) 
            -> Result<[Delta; COMP], crate::Error> {
        let mut res = [Delta::ZERO; COMP];
        for i in 0 .. COMP {
            res[i] = self[i].measure(deltas[i], omegas[i], set_dist[i])?;
        }
        Ok(res)
    }

    // Async
        /// Runs [SyncComp::drive_rel_async()] for all components
        /// 
        /// # Features
        /// 
        /// Only available if the "std" feature is enabled
        #[cfg(feature = "std")]
        fn drive_rel_async(&mut self, deltas : [Delta; COMP], omegas : [Omega; COMP]) -> Result<(), crate::Error> {
            for i in 0 .. COMP {
                self[i].drive_rel_async(deltas[i], omegas[i])?;
            }
            Ok(())
        }

        /// Runs [SyncComp::drive_abs_async()] for all components
        /// 
        /// # Features
        /// 
        /// Only available if the "std" feature is enabled
        #[cfg(feature = "std")]
        fn drive_abs_async(&mut self, gamma : [Gamma; COMP], omegas : [Omega; COMP]) -> Result<(), crate::Error> {
            for i in 0 .. COMP {
                self[i].drive_abs_async(gamma[i], omegas[i])?;
            }
            Ok(())
        }   

        /// Runs [SyncComp::await_inactive()] for all components
        /// 
        /// # Features
        /// 
        /// Only available if the "std" feature is enabled
        #[cfg(feature = "std")]
        fn await_inactive(&mut self) -> Result<[Delta; COMP], crate::Error> {
            let mut delta = [Delta::NAN; COMP];

            for i in 0 .. COMP {
                delta[i] = self[i].await_inactive()?;
            }

            Ok(delta)
        }
    // 

    // Position
        /// Runs [SyncComp::gamma()] for all components
        #[inline(always)]
        fn gammas(&self) -> [Gamma; COMP] {
            let mut dists = [Gamma::ZERO; COMP];
            for i in 0 .. COMP {
                dists[i] = self[i].gamma();
            }
            dists
        }
        
        /// Runs [SyncComp::write_gamma()] for all components
        #[inline(always)]
        fn write_gammas(&mut self, gammas : &[Gamma; COMP]) {
            for i in 0 .. COMP {
                self[i].write_gamma(gammas[i])
            }
        }

        /// Runs [SyncComp::lim_for_gamma()] for all components 
        #[inline(always)]
        fn lims_for_gammas(&self, gammas : &[Gamma; COMP]) -> [Delta; COMP] {
            let mut limits = [Delta::ZERO; COMP]; 
            for i in 0 .. COMP {
                limits[i] = self[i].lim_for_gamma(gammas[i]);
            }
            limits
        }

        /// Checks if the given gammas are vaild, which means they are finite and in range of the components
        #[inline(always)]
        fn valid_gammas(&self, gammas : &[Gamma; COMP]) -> bool {
            let mut res = true;
            for i in 0 .. COMP {
                res = res & ((!self[i].lim_for_gamma(gammas[i]).is_normal()) & gammas[i].is_finite()); 
            }
            res
        }

        /// Same as [SyncCompGroup::valid_gammas()], but it evaluates the check for each component and returns seperated results for analysis
        #[inline(always)]
        fn valid_gammas_verb(&self, gammas : &[Gamma; COMP]) -> [bool; COMP] {
            let mut res = [true; COMP];
            for i in 0 .. COMP {
                res[i] = (!self[i].lim_for_gamma(gammas[i]).is_normal()) & gammas[i].is_finite(); 
            }
            res
        }

        /// Runs [SyncComp::set_end()] for all components
        #[inline(always)]
        fn set_ends(&mut self, set_dist : &[Gamma; COMP]) {
            for i in 0 .. COMP {
                self[i].set_end(set_dist[i]);
            }
        }
        
        /// Runs [SyncComp::set_limit()] for all components
        #[inline(always)]
        fn set_limits(&mut self, min : &[Option<Gamma>; COMP], max : &[Option<Gamma>; COMP]) {
            for i in 0 .. COMP {
                self[i].set_limit(min[i], max[i]);
            }
        }
    //

    // Load calculation
        /// Runs [SyncComp::apply_inertia()] for all components
        #[inline(always)]
        fn apply_inertias(&mut self, inertias : &[Inertia; COMP]) {
            for i in 0 .. COMP {
                self[i].apply_inertia(inertias[i]);
            }
        }

        /// Runs [SyncComp::apply_force()] for all components
        #[inline(always)]
        fn apply_forces(&mut self, forces : &[Force; COMP]) {
            for i in 0 .. COMP {
                self[i].apply_force(forces[i]);
            }
        }

        /// Runs [SyncComp::apply_bend_f()] for all components. Note that the same factor is applied to all components
        #[inline(always)]
        fn apply_bend_f(&mut self, f_bend : f32) {
            for i in 0 .. COMP {
                self[i].apply_bend_f(f_bend);
            }
        }
    // 
}

// Implementations
impl<const N : usize> SyncCompGroup<dyn SyncComp, N> for [Box<dyn SyncComp>; N] { }
impl<const N : usize> SyncCompGroup<dyn SyncComp, N> for Vec<Box<dyn SyncComp>> { }
