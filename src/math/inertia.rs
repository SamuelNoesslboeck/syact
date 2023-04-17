use alloc::vec::Vec;

use glam::{Vec3, Mat3};

use crate::units::*;

/// Calculates the inertia of a given point with a certain distance `dist` to a center point and a certain `mass`
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

/// Helper function, calculates the inertia of a rod construction
fn inertia_rod_constr_coord(constr : &Vec<RodCoord>) -> Inertia {
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

/// Calculates the inertia of a rod construction
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

/// Reduces a given `inertia` matrix to a mass scalar in the direction of a vector `a_hat` (does not have to be a unit vector)
/// with the `radius` to the rotation center of the inertia
pub fn inertia_to_mass(inertia : Mat3, radius : Vec3, mut a_hat : Vec3) -> Inertia {
    a_hat = a_hat.normalize();

    let eta = radius.cross(a_hat);
    
    Inertia((inertia * (eta/eta.length().powi(3))).length())
}