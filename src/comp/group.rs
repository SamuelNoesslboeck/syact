use alloc::boxed::Box;
use alloc::vec::Vec;

use core::ops::IndexMut;

use crate::SyncComp;
use crate::units::*;

pub trait ComponentGroup<T, const N : usize> : IndexMut<usize, Output = Box<T>> 
    where
        T: SyncComp,
        T: ?Sized
{
    // Data
        fn link_all(&mut self, lk : crate::data::LinkedData) {
            for i in 0 .. N {
                self[i].write_link(lk.clone())
            }
        }
    //

    fn drive_rel(&mut self, deltas : [Delta; N], omegas : [Omega; N]) -> Result<[Delta; N], crate::Error> {
        let mut res = [Delta::ZERO; N];
        for i in 0 .. N {
            res[i] = self[i].drive_rel(deltas[i], omegas[i])?;
        }
        Ok(res)
    }

    fn drive_abs(&mut self, gamma : [Gamma; N], omegas : [Omega; N]) -> Result<[Delta; N], crate::Error>  {
        let mut res = [Delta::ZERO; N];
        for i in 0 .. N {
            res[i] = self[i].drive_abs(gamma[i], omegas[i])?;
        }
        Ok(res)
    }

    fn measure(&mut self, deltas : [Delta; N], omegas : [Omega; N], set_dist : [Gamma; N]) 
            -> Result<[Delta; N], crate::Error> {
        let mut res = [Delta::ZERO; N];
        for i in 0 .. N {
            res[i] = self[i].measure(deltas[i], omegas[i], set_dist[i])?;
        }
        Ok(res)
    }

    // Position
        fn gammas(&self) -> [Gamma; N] {
            let mut dists = [Gamma::ZERO; N];
            for i in 0 .. N {
                dists[i] = self[i].gamma();
            }
            dists
        }
        
        fn write_gammas(&mut self, gammas : &[Gamma; N]) {
            for i in 0 .. N {
                self[i].write_gamma(gammas[i])
            }
        }

        fn lims_for_gammas(&self, gammas : &[Gamma; N]) -> [Delta; N] {
            let mut limits = [Delta::ZERO; N]; 
            for i in 0 .. N {
                limits[i] = self[i].lim_for_gamma(gammas[i]);
            }
            limits
        }

        fn valid_gammas(&self, gammas : &[Gamma; N]) -> bool {
            let mut res = true;
            for i in 0 .. N {
                res = res & ((!self[i].lim_for_gamma(gammas[i]).is_normal()) & gammas[i].is_finite()); 
            }
            res
        }

        fn valid_gammas_verb(&self, gammas : &[Gamma; N]) -> [bool; N] {
            let mut res = [true; N];
            for i in 0 .. N {
                res[i] = (!self[i].lim_for_gamma(gammas[i]).is_normal()) & gammas[i].is_finite(); 
            }
            res
        }

        fn set_end(&mut self, set_dist : &[Gamma; N]) {
            for i in 0 .. N {
                self[i].set_end(set_dist[i]);
            }
        }

        fn set_limit(&mut self, min : &[Option<Gamma>; N], max : &[Option<Gamma>; N]) {
            for i in 0 .. N {
                self[i].set_limit(min[i], max[i]);
            }
        }
    //

    // Load calculation
        fn apply_inertia(&mut self, inertias : &[Inertia; N]) {
            for i in 0 .. N {
                self[i].apply_inertia(inertias[i]);
            }
        }

        fn apply_forces(&mut self, forces : &[Force; N]) {
            for i in 0 .. N {
                self[i].apply_force(forces[i]);
            }
        }
    // 
}

// Implementations
impl<const N : usize> ComponentGroup<dyn SyncComp, N> for [Box<dyn SyncComp>; N] { }
impl<const N : usize> ComponentGroup<dyn SyncComp, N> for Vec<Box<dyn SyncComp>> { }
