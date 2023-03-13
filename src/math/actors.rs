use alloc::boxed::Box;

use crate::SyncComp;
use crate::units::*;

pub fn deltas<const N : usize>(pos_0 : [Gamma; N], pos : [Gamma; N]) -> [Delta; N] {
    let mut deltas = [Delta::ZERO; N];
    for i in 0 .. N {
        deltas[i] = Delta::diff(pos_0[i], pos[i]);
    }
    deltas
}

/// Returns an array of [ [ t_min, t_max ], [vel exit case min]]
pub fn compl_times<const N : usize>(comps : &[Box<dyn SyncComp>; N], 
        pos_0 : [Gamma; N], pos : [Gamma; N], vel_0 : [Omega; N], vel_max : [Omega; N]) -> [([Time; 2], [Omega; 2], [Alpha; 2]); N] {
    let mut res = [([Time::ZERO; 2], [Omega::ZERO; 2], [Alpha::ZERO; 2]); N]; 
    for i in 0 .. N {
        res[i] = comps[i].compl_times(pos_0[i], pos[i] - pos_0[i], vel_0[i], vel_max[i]);
    }
    res
}

/// Returns [ f_s, index max of t_min, index min of t_max ]
pub fn f_s<const N : usize>(res : &[([Time; 2], [Omega; 2], [Alpha; 2]); N]) -> (f32, usize, usize) {
    // Highest of all minimum required times
    let mut t_min_max = Time::ZERO;
    // Lowest of all maximum allowed times
    let mut t_max_min = Time::INFINITY;

    let mut t_min_max_index : usize = 0;
    let mut t_max_min_index : usize = 0;

    for i in 0 .. N {
        let [ t_min, t_max ] = res[i].0;

        if (t_min > t_min_max) & t_min.is_finite() {
            t_min_max = t_min;
            t_min_max_index = i;
        }
        
        if (t_max < t_max_min) & t_max.is_finite() {
            t_max_min = t_max;
            t_max_min_index = i;
        }
    }

    ( t_max_min / t_min_max, t_min_max_index, t_max_min_index )
}