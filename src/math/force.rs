use core::f32::consts::{E, PI};

use alloc::vec::Vec;
use glam::Vec3;

use crate::StepperConst;
use crate::units::*;

/// Returns the current torque [Force] that a DC-Motor can produce when driving with the speed `omega` and the voltage `u` in Volts
pub fn torque_dyn(data : &StepperConst, mut omega : Omega, u : f32) -> Force {
    omega = omega.abs();
    
    if !omega.is_normal() {
        return data.t_s;
    }

    let t = 2.0 * PI / (data.n_c as f32) / omega;
    let pow = E.powf( -t / data.tau(u) );

    (1.0 - pow) / (1.0 + pow) * data.t_s
}

pub fn forces_segment(actions : &Vec<(Vec3, Vec3)>, mut torque : Vec3, ac : Vec3, ac_hat : Vec3) -> (Vec3, Vec3) {
    let eta_ = ac.cross(ac_hat);
    let eta = eta_ / eta_.length().powi(2);

    let mut f_j = Vec3::ZERO;

    for (f_a, a_f) in actions {
        torque += a_f.cross(*f_a);
        f_j += *f_a;
    }

    let f_c = torque.dot(eta) * ac_hat;

    (f_c, f_j - f_c) 
}

pub fn forces_joint(actions : &Vec<(Vec3, Vec3)>, mut torque : Vec3) -> (Vec3, Vec3) {
    let mut f_j = Vec3::ZERO;

    for (f_a, a_f) in actions {
        torque += a_f.cross(*f_a);
        f_j += *f_a; 
    }

    (torque, f_j)
}