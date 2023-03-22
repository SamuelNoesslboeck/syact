use alloc::boxed::Box;
use alloc::vec::Vec;

use core::ops::Index;
use core::ops::IndexMut;

use crate::SyncComp;
use crate::units::*;

pub trait SyncCompGroup<T, const COMP : usize> : IndexMut<usize, Output = Box<T>> + Index<usize, Output = Box<T>>
    where
        T: SyncComp,
        T: ?Sized
{
    // Setup
        fn setup(&mut self) {
            for i in 0 .. COMP {
                self[i].setup();
            }
        }

        #[cfg(feature = "std")]
        fn setup_async(&mut self) {
            for i in 0 .. COMP {
                self[i].setup_async();
            }
        }
    // 

    // Data
        fn link(&mut self, lk : crate::data::LinkedData) {
            for i in 0 .. COMP {
                self[i].write_link(lk.clone())
            }
        }
    //

    fn drive_rel(&mut self, deltas : [Delta; COMP], omegas : [Omega; COMP]) -> Result<[Delta; COMP], crate::Error> {
        let mut res = [Delta::ZERO; COMP];
        for i in 0 .. COMP {
            res[i] = self[i].drive_rel(deltas[i], omegas[i])?;
        }
        Ok(res)
    }

    fn drive_abs(&mut self, gamma : [Gamma; COMP], omegas : [Omega; COMP]) -> Result<[Delta; COMP], crate::Error>  {
        let mut res = [Delta::ZERO; COMP];
        for i in 0 .. COMP {
            res[i] = self[i].drive_abs(gamma[i], omegas[i])?;
        }
        Ok(res)
    }

    fn measure(&mut self, deltas : [Delta; COMP], omegas : [Omega; COMP], set_dist : [Gamma; COMP]) 
            -> Result<[Delta; COMP], crate::Error> {
        let mut res = [Delta::ZERO; COMP];
        for i in 0 .. COMP {
            res[i] = self[i].measure(deltas[i], omegas[i], set_dist[i])?;
        }
        Ok(res)
    }

    // Async
        #[cfg(feature = "std")]
        fn drive_rel_async(&mut self, deltas : [Delta; COMP], omegas : [Omega; COMP]) -> Result<(), crate::Error> {
            for i in 0 .. COMP {
                self[i].drive_rel_async(deltas[i], omegas[i])?;
            }
            Ok(())
        }

        #[cfg(feature = "std")]
        fn drive_abs_async(&mut self, gamma : [Gamma; COMP], omegas : [Omega; COMP]) -> Result<(), crate::Error> {
            for i in 0 .. COMP {
                self[i].drive_abs_async(gamma[i], omegas[i])?;
            }
            Ok(())
        }

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
        #[inline(always)]
        fn gammas(&self) -> [Gamma; COMP] {
            let mut dists = [Gamma::ZERO; COMP];
            for i in 0 .. COMP {
                dists[i] = self[i].gamma();
            }
            dists
        }
        
        #[inline(always)]
        fn write_gammas(&mut self, gammas : &[Gamma; COMP]) {
            for i in 0 .. COMP {
                self[i].write_gamma(gammas[i])
            }
        }

        #[inline(always)]
        fn lims_for_gammas(&self, gammas : &[Gamma; COMP]) -> [Delta; COMP] {
            let mut limits = [Delta::ZERO; COMP]; 
            for i in 0 .. COMP {
                limits[i] = self[i].lim_for_gamma(gammas[i]);
            }
            limits
        }

        #[inline(always)]
        fn valid_gammas(&self, gammas : &[Gamma; COMP]) -> bool {
            let mut res = true;
            for i in 0 .. COMP {
                res = res & ((!self[i].lim_for_gamma(gammas[i]).is_normal()) & gammas[i].is_finite()); 
            }
            res
        }

        #[inline(always)]
        fn valid_gammas_verb(&self, gammas : &[Gamma; COMP]) -> [bool; COMP] {
            let mut res = [true; COMP];
            for i in 0 .. COMP {
                res[i] = (!self[i].lim_for_gamma(gammas[i]).is_normal()) & gammas[i].is_finite(); 
            }
            res
        }

        #[inline(always)]
        fn set_end(&mut self, set_dist : &[Gamma; COMP]) {
            for i in 0 .. COMP {
                self[i].set_end(set_dist[i]);
            }
        }

        #[inline(always)]
        fn set_limit(&mut self, min : &[Option<Gamma>; COMP], max : &[Option<Gamma>; COMP]) {
            for i in 0 .. COMP {
                self[i].set_limit(min[i], max[i]);
            }
        }
    //

    // Load calculation
        #[inline(always)]
        fn apply_inertias(&mut self, inertias : &[Inertia; COMP]) {
            for i in 0 .. COMP {
                self[i].apply_inertia(inertias[i]);
            }
        }

        #[inline(always)]
        fn apply_forces(&mut self, forces : &[Force; COMP]) {
            for i in 0 .. COMP {
                self[i].apply_force(forces[i]);
            }
        }

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
