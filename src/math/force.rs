use core::f32::consts::{E, PI};

use alloc::vec::Vec;
use glam::Vec3;

use crate::StepperConst;
use crate::units::*;

/// Returns the current torque [Force] that a DC-Motor can produce when driving with the speed `omega` and the voltage `u` in Volts
/// 
/// # Panics
/// 
/// Panics if the given omega is not finite
pub fn torque_dyn(data : &StepperConst, mut omega : Omega, u : f32) -> Force {
    omega = omega.abs();

    if !omega.is_finite() {
        panic!("Bad omega! {}", omega);
    }
    
    if omega == Omega::ZERO {
        return data.t_s;
    }

    let t = 2.0 * PI / (data.n_c as f32) / omega;
    let pow = E.powf( -t / data.tau(u) );

    (1.0 - pow) / (1.0 + pow) * data.t_s
}

/// Calculates all forces and torques acting upon a segment connected with another support segment
/// 
/// - `actions` is a tuple vector of all forces acting upon the segment, each tuple consists of (`f_a`, `a_f`)
///     - `f_a` being the actual force acting
///     - `a_f` being the positional vector of the force
/// - `torque` is the torque acting on the center point
/// - `as` is the position of the segment support that creates the reaction force 
/// - `a_hat` is the direction of this support
/// 
/// Returns the two resulting forces (`f_c`, `f_j`)
/// - `f_s` being the force acting on the support segment 
/// - `f_j` being the force acting on the joint (center point, position reference point)
pub fn forces_segment(actions : &Vec<(Vec3, Vec3)>, mut torque : Vec3, a_s : Vec3, a_hat : Vec3) -> (Vec3, Vec3) {
    let eta_ = a_s.cross(a_hat);
    let eta = eta_ / eta_.length().powi(2);

    let mut f_j = Vec3::ZERO;

    for (f_a, a_f) in actions {
        torque += a_f.cross(*f_a);
        f_j += *f_a;
    }

    let f_s = torque.dot(eta) * a_hat;

    (f_s, f_j - f_s) 
}

/// Calculates all the forces in a joint without any support 
/// 
/// - `actions` is a tuple vector of all forces acting upon the segment, each tuple consists of (`f_a`, `a_f`)
///     - `f_a` being the actual force acting
///     - `a_f` being the positional vector of the force
/// - `torque` is the torque acting on the center point
/// 
/// Returns the resulting torque and the resulting joint force (`torque`, `f_j`)
pub fn forces_joint(actions : &Vec<(Vec3, Vec3)>, mut torque : Vec3) -> (Vec3, Vec3) {
    let mut f_j = Vec3::ZERO;

    for (f_a, a_f) in actions {
        torque += a_f.cross(*f_a);
        f_j += *f_a; 
    }

    (torque, f_j)
}