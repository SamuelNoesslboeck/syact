use alloc::vec::Vec;

use glam::Vec3;

/// Calculates all forces for a segment
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