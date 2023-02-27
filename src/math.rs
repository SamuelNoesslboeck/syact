extern crate alloc;
use alloc::boxed::Box;
use alloc::vec::Vec;

use core::f32::consts::{E, PI};

use glam::{Vec3, Mat3};

use crate::{Component, Omega, Gamma, Alpha, Delta, Force, Time, Inertia};
use crate::data::{StepperConst, StepperVar};

/// Returns the current torque of a motor (data) at the given angluar speed (omega), returns only positive values  \
/// Unit: [Nm]  
pub fn torque_dyn(data : &StepperConst, mut omega : Omega, u : f32) -> Force {
    omega = omega.abs();
    
    if omega == Omega::ZERO {
        return data.t_s;
    }

    let t = 2.0 * PI / (data.n_c as f32) / omega;
    let tau = data.tau(u);
    let pow = E.powf( -t / tau );

    return (1.0 - pow) / (1.0 + pow) * data.t_s;
}

/// Returns the start freqency of a motor (data)  \
/// Unit: [Hz]
pub fn start_frequency(data : &StepperConst, var : &StepperVar) -> f32 {
    (data.alpha_max(var) * (data.n_s as f32) / 4.0 / PI).0.powf(0.5)
}

/// The angluar velocity of a motor that is constantly accelerating after the time `t`
pub fn angluar_velocity(data : &StepperConst, var : &StepperVar, t : Time, u : f32) -> Omega {
    return data.alpha_max(var) * (t + data.tau(u)*E.powf(-t/data.tau(u)));
}

/// The angluar velocity of a motor that is constantly accelerating after the time t [in s], [in s^-1]
pub fn angluar_velocity_dyn(data : &StepperConst, var : &StepperVar, t : Time, omega_approx : Omega, u : f32) -> Omega {
    data.alpha_max_dyn(torque_dyn(data, omega_approx, u), var) * (t + data.tau(u)*E.powf(-t/data.tau(u)))
}


// || Inertias ||
    pub fn inertia_point(dist : Vec3, mass : f32) -> Mat3 {
        return mass * Mat3 {
            x_axis: Vec3 { x: dist.y.powi(2) + dist.z.powi(2), y: 0.0, z: 0.0 }, 
            y_axis: Vec3 { x: 0.0, y: dist.x.powi(2) + dist.z.powi(2), z: 0.0 },
            z_axis: Vec3 { x: 0.0, y: 0.0, z: dist.x.powi(2) + dist.y.powi(2) }
        }; 
    }

    // Construction
    /// Rod helper type, consists of (mass : f32, vec : Vec3)
    pub type Rod = (f32, Vec3); 
    /// Rod helper type for coords, consists of (mass : f32, coord : f32)
    pub type RodCoord = (f32, f32); 

    pub fn inertia_rod_constr_coord(constr : &Vec<RodCoord>) -> Inertia {
        let mut inertia = 0.0;

        for i in 0 .. constr.len() {
            let mut len_sum = 0.0;

            for j in 0 .. i {
                len_sum += constr[j].1;
            }

            inertia += constr[i].0 * (constr[i].1.powi(2) / 12.0 + (len_sum + constr[i].1 / 2.0).powi(2));
        }

        Inertia(inertia)
    }

    pub fn inertia_rod_constr(constr : &Vec<Rod>) -> Mat3 {
        let mut x_list : Vec<RodCoord> = alloc::vec![];
        for rod in constr {
            x_list.push((rod.0, rod.1.x));
        }

        let mut y_list : Vec<RodCoord> = alloc::vec![];
        for rod in constr {
            y_list.push((rod.0, rod.1.y));
        }

        let mut z_list : Vec<RodCoord> = alloc::vec![];
        for rod in constr {
            z_list.push((rod.0, rod.1.z));
        }

        let j_x : f32 = inertia_rod_constr_coord(&x_list).into();
        let j_y : f32 = inertia_rod_constr_coord(&y_list).into();
        let j_z : f32 = inertia_rod_constr_coord(&z_list).into();

        Mat3 { 
            x_axis: Vec3 { x: (j_y + j_z), y: 0.0, z: 0.0 }, 
            y_axis: Vec3 { x: 0.0, y: (j_x + j_z), z: 0.0 }, 
            z_axis: Vec3 { x: 0.0, y: 0.0, z: (j_x + j_y) } 
        }
    }

    pub fn inertia_to_mass(inertia : Mat3, radius : Vec3, mut a_hat : Vec3) -> Inertia {
        a_hat = a_hat.normalize();

        let eta = radius.cross(a_hat);
        
        Inertia((inertia * (eta/eta.length().powi(3))).length())
    }
//

// || Forces || 
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
//

// Helper
fn pq_formula_times(p : f32, q : f32) -> (Time, Time) {
    ( Time(-p/2.0 + ((p/2.0).powi(2) - q).sqrt()), Time(-p/2.0 - ((p/2.0).powi(2) - q).sqrt() ))
}

#[inline]
fn correct_time(t : Time) -> Time {
    if t.0.is_nan() { Time(f32::INFINITY) } else { if t <= Time::ZERO { Time(f32::INFINITY) } else { t } } 
}

fn correct_times(times : (Time, Time)) -> (Time, Time) {
    ( correct_time(times.0), correct_time(times.1) )
}
// 

/// Trait for advanced calculations of mechanical actors
pub trait MathActor
{
    fn accel_dyn(&self, omega : Omega, gamma : Gamma) -> Alpha;

    fn accel_max_node(&self, gamma_0 : Gamma, delta_pos : Delta, omega_0 : Omega, vel_max : Omega) -> (Alpha, Alpha) {
        // Get maximum accelerations
        let ( t_pos, mut accel_max_pos ) = self.node_from_vel(delta_pos, omega_0, vel_max.abs());
        let ( t_neg, mut accel_max_neg ) = self.node_from_vel(delta_pos, omega_0, -(vel_max.abs()));

        let accel = self.accel_dyn(((omega_0 + vel_max) / 2.0).abs(), gamma_0);

        if !t_pos.0.is_finite() { 
            accel_max_pos = accel;
        }

        if !t_neg.0.is_finite() {
            accel_max_neg = -accel;
        }

        ( accel.min(accel_max_pos), (-accel).max(accel_max_neg) )
    }

    fn node_from_vel(&self, delta_pos : Delta, omega_0 : Omega, omega : Omega) -> (Time, Alpha) {
        let time = correct_time(2.0 * delta_pos / (omega_0 + omega));
        ( time, (omega - omega_0) / time )
    }

    /// Returns ([t_min, t_max], [vel exit case min, vel exit case max], [accel exit case min, accel exit case max])  
    fn compl_times(&self, gamma_0 : Gamma, delta_pos : Delta, omega_0 : Omega, omega_max : Omega) -> ([Time; 2], [Omega; 2], [Alpha; 2]) {
        let ( accel_pos, accel_neg ) = self.accel_max_node(gamma_0, delta_pos, omega_0, omega_max); 

        let ( t_pos_1, t_pos_2 ) = correct_times(pq_formula_times(2.0 * omega_0.0 / accel_pos.0, -2.0 * delta_pos.0 / accel_pos.0));
        let ( t_neg_1, t_neg_2 ) = correct_times(pq_formula_times(2.0 * omega_0.0 / accel_neg.0, -2.0 * delta_pos.0 / accel_neg.0));

        let time;
        let vels;
        let accel;

        let mut t_pos = t_pos_1.min(t_pos_2);
        let mut t_neg = t_neg_1.min(t_neg_2);

        let t_pos_max = t_pos; // t_pos_1.max(t_pos_2);
        let t_neg_max = t_neg;  // t_neg_1.max(t_neg_2);

        if accel_pos == Alpha::ZERO {
            t_pos = correct_time(delta_pos / omega_0);
        }

        if accel_neg == Alpha::ZERO {
            t_neg = correct_time(delta_pos / omega_0);
        }

        if t_pos <= t_neg {
            time = [ t_pos, t_neg_max ];
            vels = [ omega_0 + t_pos * accel_pos, omega_0 + t_neg_max * accel_neg ];
            accel = [ accel_pos, accel_neg ];
        } else {
            time = [ t_neg, t_pos_max ];
            vels = [ omega_0 + t_neg * accel_neg, omega_0 + t_pos_max * accel_pos ];
            accel = [ accel_neg, accel_pos ];
        }
        
        ( time, vels, accel )
    }
}

pub mod actors 
{
    use super::*;

    pub fn deltas<const N : usize>(pos_0 : [Gamma; N], pos : [Gamma; N]) -> [Delta; N] {
        let mut deltas = [Delta::ZERO; N];
        for i in 0 .. N {
            deltas[i] = Delta::diff(pos_0[i], pos[i]);
        }
        deltas
    }

    /// Returns an array of [ [ t_min, t_max ], [vel exit case min]]
    pub fn compl_times<const N : usize>(comps : &[Box<dyn Component>; N], pos_0 : [Gamma; N], pos : [Gamma; N], vel_0 : [Omega; N], vel_max : [Omega; N]) -> [([Time; 2], [Omega; 2], [Alpha; 2]); N] {
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
}