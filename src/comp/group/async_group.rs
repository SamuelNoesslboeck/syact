use core::ops::IndexMut;

use crate::comp::asynchr::AsyncComp;
use crate::units::*;

pub trait AsyncCompGroup<const N : usize> : IndexMut<usize, Output = Box<dyn AsyncComp>> {
    fn drive_rel_async(&mut self, deltas : [Delta; N], omegas : [Omega; N]) {
        for i in 0 .. N {
            self[i].drive_rel_async(deltas[i], omegas[i]);
        }
    }

    fn drive_abs_async(&mut self, gammas : [Gamma; N], omegas : [Omega; N]) {
        for i in 0 .. N {
            self[i].drive_abs_async(gammas[i], omegas[i]);
        }
    }

    fn measure_async(&mut self, deltas : [Delta; N], omegas : [Omega; N], set_gammas : [Gamma; N], accuracy : [u64; N]) {
        for i in 0 .. N {
            self[i].measure_async(deltas[i], omegas[i], set_gammas[i], accuracy[i])
        }
    }

    fn await_inactive(&self) {
        for i in 0 .. N {
            self[i].await_inactive();
        }
    }
}