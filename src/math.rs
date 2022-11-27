use std::{f32::consts::{E, PI}, ops::Index};

use glam::{Vec3, Mat3};

use super::data::StepperData;

/// Returns the current torque of a motor (data) at the given angluar speed (omega)  \
/// Unit: [Nm]  
pub fn torque(data : &StepperData, omega : f32) -> f32 {
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
    return (data.t() / data.j() * (data.n_s as f32) / 4.0 / PI).powf(0.5);
}

/// The angluar velocity of a motor that is constantly accelerating after the time t [in s], [in s^-1]
pub fn angluar_velocity(data : &StepperData, t : f32) -> f32 {
    return data.alpha_max() * (t + data.tau()*E.powf(-t/data.tau()));
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

// Inertias
pub fn inertia_rod(length : f32, mass : f32) -> f32 {
    length.powi(2) * mass / 12.0
}

pub fn inertia_rod_end(length : f32, mass : f32) -> f32 {
    length.powi(2) * mass / 3.0
}

pub fn inertia_point(dist : f32, mass : f32) -> f32 {
    dist.powi(2) * mass
}

pub fn inertia_rot_rod(length : f32, dist : f32, mass : f32) -> f32 {
    inertia_rod(length, mass) + inertia_point(dist, mass)
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

    Mat3 { 
        x_axis: Vec3 { x: inertia_rod_constr_coord(&x_list), y: 0.0, z: 0.0 }, 
        y_axis: Vec3 { x: 0.0, y: inertia_rod_constr_coord(&y_list), z: 0.0 }, 
        z_axis: Vec3 { x: 0.0, y: 0.0, z: inertia_rod_constr_coord(&z_list) } 
    }
}