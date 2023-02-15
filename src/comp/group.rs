use std::ops::IndexMut;

use crate::Component;

pub type Gammas<const N : usize> = [f32; N]; 
pub type Omegas<const N : usize> = [f32; N];

pub type Inertias<const N : usize> = [f32; N];
pub type Forces<const N : usize> = [f32; N]; 

pub trait ComponentGroup<const N : usize> : IndexMut<usize, Output = Box<dyn Component>>
{
    // Data
        fn link_all(&mut self, lk : std::sync::Arc<crate::LinkedData>) {
            for i in 0 .. N {
                self[i].link(lk.clone())
            }
        }
    //

    fn drive_rel(&mut self, dist : [f32; N], vel : Omegas<N>) -> [f32; N] {
        let mut res = [0.0; N];
        for i in 0 .. N {
            res[i] = self[i].drive_rel(dist[i], vel[i]);
        }
        res
    }

    fn drive_abs(&mut self, dist : Gammas<N>, vel : Omegas<N>) -> [f32; N] {
        let mut res = [0.0; N];
        for i in 0 .. N {
            res[i] = self[i].drive_rel(dist[i], vel[i]);
        }
        res
    }

    fn drive_rel_async(&mut self, dist : [f32; N], vel : Omegas<N>) {
        for i in 0 .. N {
            self[i].drive_rel_async(dist[i], vel[i]);
        }
    }

    fn drive_abs_async(&mut self, dist : Gammas<N>, vel : Omegas<N>) {
        for i in 0 .. N {
            self[i].drive_abs_async(dist[i], vel[i]);
        }
    }

    fn measure(&mut self, dist : [f32; N], vel : [f32; N], set_dist : Gammas<N>, accuracy : [u64; N]) -> [bool; N] {
        let mut res = [false; N];
        for i in 0 .. N {
            res[i] = self[i].measure(dist[i], vel[i], set_dist[i], accuracy[i])
        }
        res
    }

    fn measure_async(&mut self, dist : [f32; N], vel : Omegas<N>, accuracy : [u64; N]) {
        for i in 0 .. N {
            self[i].measure_async(dist[i], vel[i], accuracy[i])
        }
    }

    fn await_inactive(&self) {
        for i in 0 .. N {
            self[i].await_inactive();
        }
    }

    // Position
        fn get_dist(&self) -> Gammas<N> {
            let mut dists = [0.0; N];
            for i in 0 .. N {
                dists[i] = self[i].get_dist();
            }
            dists
        }
        
        fn write_dist(&mut self, angles : &Gammas<N>) {
            for i in 0 .. N {
                self[i].write_dist(angles[i])
            }
        }

        fn get_limit_dest(&self, dist : [f32; N]) -> [f32; N] {
            let mut limits = [0.0; N]; 
            for i in 0 .. N {
                limits[i] = self[i].get_limit_dest(dist[i]);
            }
            limits
        }

        fn valid_dist(&self, dist : &Gammas<N>) -> bool {
            let mut res = true;
            for i in 0 .. N {
                res = res & ((!self[i].get_limit_dest(dist[i]).is_normal()) & dist[i].is_finite()); 
            }
            res
        }

        fn valid_dist_verb(&self, dist : &Gammas<N>) -> [bool; N] {
            let mut res = [true; N];
            for i in 0 .. N {
                res[i] = (!self[i].get_limit_dest(dist[i]).is_normal()) & dist[i].is_finite(); 
            }
            res
        }

        fn set_endpoint(&mut self, set_dist : &Gammas<N>) -> [bool; N] {
            let mut res = [false; N];
            for i in 0 .. N {
                res[i] = self[i].set_endpoint(set_dist[i]);
            }   
            res
        }

        fn set_limit(&mut self, limit_min : &[Option<f32>; N], limit_max : &[Option<f32>; N]) {
            for i in 0 .. N {
                self[i].set_limit(limit_min[i], limit_max[i]);
            }
        }
    //

    // Load calculation
        fn apply_load_inertias(&mut self, inertias : &Inertias<N>) {
            for i in 0 .. N {
                self[i].apply_load_inertia(inertias[i]);
            }
        }

        fn apply_load_forces(&mut self, forces : &Forces<N>) {
            for i in 0 .. N {
                self[i].apply_load_force(forces[i]);
            }
        }
    // 
}

// Implementations
impl<const N : usize> ComponentGroup<N> for [Box<dyn Component>; N] { }
impl<const N : usize> ComponentGroup<N> for Vec<Box<dyn Component>> { }