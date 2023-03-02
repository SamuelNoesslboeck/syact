extern crate alloc;
use alloc::boxed::Box;
use alloc::sync::Arc;
use alloc::vec::Vec;

use core::ops::IndexMut;

use crate::{Component, Gamma, Delta, Omega, Inertia, Force};

pub trait ComponentGroup<const N : usize> : IndexMut<usize, Output = Box<dyn Component>>
{
    // Data
        fn link_all(&mut self, lk : Arc<crate::LinkedData>) {
            for i in 0 .. N {
                self[i].link(lk.clone())
            }
        }
    //

    fn drive_rel(&mut self, deltas : [Delta; N], omegas : [Omega; N]) -> [Delta; N] {
        let mut res = [Delta::ZERO; N];
        for i in 0 .. N {
            res[i] = self[i].drive_rel(deltas[i], omegas[i]);
        }
        res
    }

    fn drive_rel_async(&mut self, deltas : [Delta; N], omegas : [Omega; N]) {
        for i in 0 .. N {
            self[i].drive_rel_async(deltas[i], omegas[i]);
        }
    }

    fn drive_abs(&mut self, gamma : [Gamma; N], omegas : [Omega; N]) -> [Delta; N] {
        let mut res = [Delta::ZERO; N];
        for i in 0 .. N {
            res[i] = self[i].drive_abs(gamma[i], omegas[i]);
        }
        res
    }

    fn drive_abs_async(&mut self, gammas : [Gamma; N], omegas : [Omega; N]) {
        for i in 0 .. N {
            self[i].drive_abs_async(gammas[i], omegas[i]);
        }
    }

    fn measure(&mut self, deltas : [Delta; N], omegas : [Omega; N], set_dist : [Gamma; N], accuracy : [u64; N]) -> [bool; N] {
        let mut res = [false; N];
        for i in 0 .. N {
            res[i] = self[i].measure(deltas[i], omegas[i], set_dist[i], accuracy[i])
        }
        res
    }

    fn measure_async(&mut self, deltas : [Delta; N], omegas : [Omega; N], accuracy : [u64; N]) {
        for i in 0 .. N {
            self[i].measure_async(deltas[i], omegas[i], accuracy[i])
        }
    }

    fn await_inactive(&self) {
        for i in 0 .. N {
            self[i].await_inactive();
        }
    }

    // Position
        fn get_gammas(&self) -> [Gamma; N] {
            let mut dists = [Gamma::ZERO; N];
            for i in 0 .. N {
                dists[i] = self[i].get_gamma();
            }
            dists
        }
        
        fn write_gammas(&mut self, gammas : &[Gamma; N]) {
            for i in 0 .. N {
                self[i].write_gamma(gammas[i])
            }
        }

        fn get_limit_dest(&self, gammas : &[Gamma; N]) -> [Delta; N] {
            let mut limits = [Delta::ZERO; N]; 
            for i in 0 .. N {
                limits[i] = self[i].get_limit_dest(gammas[i]);
            }
            limits
        }

        fn valid_gammas(&self, gammas : &[Gamma; N]) -> bool {
            let mut res = true;
            for i in 0 .. N {
                res = res & ((!self[i].get_limit_dest(gammas[i]).is_normal()) & gammas[i].is_finite()); 
            }
            res
        }

        fn valid_gammas_verb(&self, gammas : &[Gamma; N]) -> [bool; N] {
            let mut res = [true; N];
            for i in 0 .. N {
                res[i] = (!self[i].get_limit_dest(gammas[i]).is_normal()) & gammas[i].is_finite(); 
            }
            res
        }

        fn set_endpoint(&mut self, set_dist : &[Gamma; N]) -> [bool; N] {
            let mut res = [false; N];
            for i in 0 .. N {
                res[i] = self[i].set_endpoint(set_dist[i]);
            }   
            res
        }

        fn set_limit(&mut self, min : &[Option<Gamma>; N], max : &[Option<Gamma>; N]) {
            for i in 0 .. N {
                self[i].set_limit(min[i], max[i]);
            }
        }
    //

    // Load calculation
        fn apply_load_inertias(&mut self, inertias : &[Inertia; N]) {
            for i in 0 .. N {
                self[i].apply_load_inertia(inertias[i]);
            }
        }

        fn apply_load_forces(&mut self, forces : &[Force; N]) {
            for i in 0 .. N {
                self[i].apply_load_force(forces[i]);
            }
        }
    // 
}

// Implementations
impl<const N : usize> ComponentGroup<N> for [Box<dyn Component>; N] { }
impl<const N : usize> ComponentGroup<N> for Vec<Box<dyn Component>> { }