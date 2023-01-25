use std::{f32::consts::{E, PI}, ops::Index};

use glam::{Vec3, Mat3};

use super::data::StepperData;

/// Returns the current torque of a motor (data) at the given angluar speed (omega), returns only positive values  \
/// Unit: [Nm]  
pub fn torque_dyn(data : &StepperData, mut omega : f32) -> f32 {
    omega = omega.abs();
    
    if omega == 0.0 {
        return data.t();
    }

    let t = 2.0 * PI / (data.n_c as f32) / omega;
    let tau = data.tau();
    let pow = E.powf( -t / tau );

    return (1.0 - pow) / (1.0 + pow) * data.t();
}

/// Returns the start freqency of a motor (data)  \
/// Unit: [Hz]
pub fn start_frequency(data : &StepperData) -> f32 {
    return (data.alpha_max() * (data.n_s as f32) / 4.0 / PI).powf(0.5);
}

/// The angluar velocity of a motor that is constantly accelerating after the time t [in s], [in s^-1]
pub fn angluar_velocity(data : &StepperData, t : f32) -> f32 {
    return data.alpha_max() * (t + data.tau()*E.powf(-t/data.tau()));
}

/// The angluar velocity of a motor that is constantly accelerating after the time t [in s], [in s^-1]
pub fn angluar_velocity_dyn(data : &StepperData, t : f32, omega_approx : f32) -> f32 {
    torque_dyn(data, omega_approx) / data.j() * (t + data.tau()*E.powf(-t/data.tau()))
}

/// The angluar velocity of a motor that is constantly accelerating after the time t [in s], [in s^-1]
pub fn angluar_velocity_dyn_rel(data : &StepperData, t_rel : f32, omega_approx : f32) -> f32 {
    data.alpha_max_dyn(torque_dyn(data, omega_approx)) * (t_rel)
}

/// Creates the acceleration curve for the given Stepperdata _data_, the curve can be modified by modifying the stepper data or defining a minium steptime _t min_ or a maximum length of _max len_
pub fn acc_curve(data : &StepperData, t_min : f32, max_len : u64) -> Vec<f32> {
    let mut list : Vec<f32> = vec![
        1.0 / start_frequency(data)
    ];

    let mut t_total = list[0];
    for i in 1 .. max_len {
        list.push(2.0 * PI / (data.n_s as f32) / angluar_velocity(data, t_total));
        t_total += list.index(i as usize);

        if *list.index(i as usize) < t_min {
            return list;
        }
    };

    return list;
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

    pub fn inertia_rod_constr_coord(constr : &Vec<RodCoord>) -> f32 {
        let mut inertia = 0.0;

        for i in 0 .. constr.len() {
            let mut len_sum = 0.0;

            for j in 0 .. i {
                len_sum += constr[j].1;
            }

            inertia += constr[i].0 * (constr[i].1.powi(2) / 12.0 + (len_sum + constr[i].1 / 2.0).powi(2));
        }

        inertia
    }

    pub fn inertia_rod_constr(constr : &Vec<Rod>) -> Mat3 {
        let mut x_list : Vec<RodCoord> = vec![];
        for rod in constr {
            x_list.push((rod.0, rod.1.x));
        }

        let mut y_list : Vec<RodCoord> = vec![];
        for rod in constr {
            y_list.push((rod.0, rod.1.y));
        }

        let mut z_list : Vec<RodCoord> = vec![];
        for rod in constr {
            z_list.push((rod.0, rod.1.z));
        }

        let j_x = inertia_rod_constr_coord(&x_list);
        let j_y = inertia_rod_constr_coord(&y_list);
        let j_z = inertia_rod_constr_coord(&z_list);

        Mat3 { 
            x_axis: Vec3 { x: (j_y + j_z), y: 0.0, z: 0.0 }, 
            y_axis: Vec3 { x: 0.0, y: (j_x + j_z), z: 0.0 }, 
            z_axis: Vec3 { x: 0.0, y: 0.0, z: (j_x + j_y) } 
        }
    }

    pub fn inertia_to_mass(inertia : Mat3, radius : Vec3, mut a_hat : Vec3) -> f32 {
        a_hat = a_hat.normalize();

        let eta = radius.cross(a_hat);
        
        (inertia * (eta/eta.length().powi(3))).length()
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
fn pq_formula(p : f32, q : f32) -> (f32, f32) {
    ( -p + (p.powi(2) + q).sqrt(), -p - (p.powi(2) + q).sqrt() )
}

fn correct_time(t : f32) -> f32 {
    if t.is_nan() { f32::INFINITY } else { if t < 0.0 { f32::INFINITY } else { t } } 
}

fn correct_times(times : (f32, f32)) -> (f32, f32) {
    ( correct_time(times.0), correct_time(times.1) )
}
// 

/// Trait for advanced calculations of mechanical actors
pub trait MathActor
{
    fn accel_dyn(&self, vel : f32, pos : f32) -> f32;

    fn accel_max_node(&self, pos_0 : f32, delta_pos : f32, vel_0 : f32, vel_max : f32) -> (f32, f32) {
        // Get maximum accelerations
        let ( _, accel_max_pos ) = self.node_from_vel(delta_pos, vel_0, vel_max.abs());
        let ( _, accel_max_neg ) = self.node_from_vel(delta_pos, vel_0, -(vel_max.abs()));

        let accel = self.accel_dyn(vel_0.abs(), pos_0.abs());

        ( accel.min(accel_max_pos), (-accel).max(accel_max_neg) )
    }

    /// Returns (time, acceleration)
    fn node_from_vel(&self, delta_pos : f32, vel_0 : f32, vel : f32) -> (f32, f32) {
        let time = correct_time(2.0 * delta_pos / (vel_0 + vel));
        ( time, (vel - vel_0) / time )
    }

    /// Returns ([t_min, t_max], [vel exit case min, vel exit case max])  
    /// 
    fn compl_times(&self, pos_0 : f32, delta_pos : f32, vel_0 : f32, vel_max : f32) -> [[f32; 2]; 2] {
        let ( accel_pos, accel_neg ) = self.accel_max_node(pos_0, delta_pos, vel_0, vel_max); 

        let ( t_pos_1, t_pos_2 ) = correct_times(pq_formula(2.0 * vel_0 / accel_pos, 2.0 * delta_pos / accel_pos));
        let ( t_neg_1, t_neg_2 ) = correct_times(pq_formula(2.0 * vel_0 / accel_neg, 2.0 * delta_pos / accel_neg));

        let time;
        let vels;

        let t_pos = t_pos_1.min(t_pos_2);
        let t_neg = t_neg_1.min(t_neg_2);

        if t_pos <= t_neg {
            time = [ t_pos, t_neg ];
            vels = [ vel_0 + t_pos * accel_pos, vel_0 + t_neg * accel_neg ];
        } else {
            time = [ t_neg, t_pos ];
            vels = [ vel_0 + t_neg * accel_neg, vel_0 + t_pos * accel_pos ];
        }
        
        [ time, vels ]
    }
}

pub mod actors 
{
    use std::f32::INFINITY;

    use crate::Component;

    pub fn delta_phis<const N : usize>(pos_0 : [f32; N], pos : [f32; N]) -> [f32; N] {
        let mut delta_phis = [0.0; N];
        for i in 0 .. N {
            delta_phis[i] = pos[i] - pos_0[i];
        }
        delta_phis
    }

    /// Calculate the completion times out of the two velocity endpoints
    pub fn compl_times_endpoints<const N : usize>(comps : &[Box<dyn Component>; N], delta_pos : [f32; N], vel_0 : [f32; N], vel : [f32; N]) -> [(f32, f32); N] {
        let mut res = [(0.0, 0.0); N];
        for i in 0 .. N {
            res[i] = comps[i].node_from_vel(delta_pos[i], vel_0[i], vel[i]);
        }
        res
    }

    /// Returns an array of [ [ t_min, t_max ], [vel exit case min]]
    pub fn compl_times<const N : usize>(comps : &[Box<dyn Component>; N], pos_0 : [f32; N], pos : [f32; N], vel_0 : [f32; N], relev : [f32; N], vel_max : f32) -> [[[f32; 2]; 2]; N] {
        let mut res = [[[0.0; 2]; 2]; N]; 
        for i in 0 .. N {
            res[i] = comps[i].compl_times(pos_0[i], pos[i] - pos_0[i], vel_0[i], vel_max * relev[i]);
        }
        res
    }

    /// Returns [ f_s, index max of t_min, index min of t_max ]
    pub fn f_s<const N : usize>(res : &[[[f32; 2]; 2]; N]) -> (f32, usize, usize) {
        // Highest of all minimum required times
        let mut t_min_max = 0.0;
        // Lowest of all maximum allowed times
        let mut t_max_min = INFINITY;

        let mut t_min_max_index : usize = 0;
        let mut t_max_min_index : usize = 0;

        for i in 0 .. N {
            let [ t_min, t_max ] = res[i][0];

            if t_min > t_min_max {
                t_min_max = t_min;
                t_min_max_index = i;
            }
            
            if t_max < t_max_min {
                t_max_min = t_max;
                t_max_min_index = i;
            }
        }

        ( t_max_min / t_min_max, t_min_max_index, t_max_min_index )
    }
}