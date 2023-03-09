use alloc::vec;
use alloc::vec::Vec;

use core::f32::consts::PI;

use crate::data::{StepperConst, CompVars, LinkedData};
use crate::units::*;

use crate::math::load::torque_dyn;

/// Returns the start freqency of a motor (data)  \
/// 
/// # Units
/// 
///  - Returns Hertz
#[inline]
pub fn start_frequency(data : &StepperConst, var : &CompVars) -> Omega {
    Omega((data.alpha_max(var) * (data.n_s as f32) / 4.0 / PI).0.powf(0.5))
}

#[inline]
pub fn travel_times(delta : Delta, omega : Omega, alpha : Alpha) -> (Time, Time) {
    let p = omega / alpha;
    let q = 2.0 * delta.0 / alpha.0;

    let root = Time((p.0.powi(2) + q).sqrt());

    ( -p + root, -p - root )
}

#[inline]
pub fn next_node_simple(mut delta : Delta, mut omega_0 : Omega, mut alpha : Alpha) -> (Time, Omega) {
    delta = delta.abs(); 
    omega_0 = omega_0.abs(); 
    alpha = alpha.abs(); 

    let (t_1, t_2) = travel_times(delta, omega_0, alpha);
    let t = t_1.max(t_2);

    ( t, omega_0 + alpha * t )
}

// Curves
pub fn crate_plain_curve(data : &StepperConst, node_count : usize, omega_max : Omega) -> Vec<Time> {
    vec![data.step_time(omega_max); node_count]
}

pub fn mirror_curve(cur : &mut [Time]) {
    let cur_len = cur.len();

    for i in 0 .. cur_len / 2 {
        cur[cur_len - i - 1] = cur[i];
    }
}

pub fn write_simple_move(cur : &mut [Time], omega_max : Omega, data : &StepperConst, var : &CompVars, lk : &LinkedData) {
    let cur_len = cur.len(); 

    let delta = data.step_ang();

    let mut time : Time;
    let mut omega = Omega::ZERO;
    let mut alpha : Alpha;

    for i in 0 .. cur_len / 2 {
        alpha = data.alpha_max_dyn(torque_dyn(data, omega, lk.u), var) / lk.s_f;
        
        (time, omega) = next_node_simple(delta, omega, alpha);

        if omega > omega_max {
            break;
        }

        cur[i] = time;
    }

    mirror_curve(cur);
}

#[inline]
pub fn create_simple_curve(consts : &StepperConst, vars : &CompVars, lk : &LinkedData, delta : Delta, omega_max : Omega) 
        -> Vec<Time> {
    let steps = consts.steps_from_ang(delta);

    let mut cur = crate_plain_curve(consts, steps as usize, omega_max);
    write_simple_move(cur.as_mut_slice(), omega_max, consts, vars, lk);

    cur
}