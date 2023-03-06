use crate::comp::asyn::AsyncComp;
use crate::{units::*, ComponentGroup};

pub trait AsyncCompGroup<T, const N : usize> : ComponentGroup<T, N> 
    where
        T: AsyncComp,
        T: ?Sized
{
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

impl<const N : usize> ComponentGroup<dyn AsyncComp, N> for [Box<dyn AsyncComp>; N] { }
impl<const N : usize> ComponentGroup<dyn AsyncComp, N> for Vec<Box<dyn AsyncComp>> { }

impl<const N : usize> AsyncCompGroup<dyn AsyncComp, N> for [Box<dyn AsyncComp>; N] { }
impl<const N : usize> AsyncCompGroup<dyn AsyncComp, N> for Vec<Box<dyn AsyncComp>> { }