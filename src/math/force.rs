use core::f32::consts::E;

use alloc::vec::Vec;
use glam::Vec3;

use crate::StepperConst;
use crate::units::*;

/// Returns the current torque [Force] that a DC-Motor can produce when driving with the 
/// speed `omega` and the voltage `u` in Volts
/// 
/// # Panics
/// 
/// Panics if the given omega is not finite
/// 
/// ```rust
/// use syact::StepperConst;
/// use syact::math::force::torque_dyn;
/// use syact::units::*;
/// 
/// let data = StepperConst::GEN;   // Using generic stepper motor data
/// 
/// // Without speed (in stall) the torque has to equal the stall torque
/// assert_eq!(torque_dyn(&data, Omega::ZERO, 12.0, 1), data.t_s);    
/// // The torque has to reduce with increasing speed 
/// assert!(torque_dyn(&data, Omega(10.0), 12.0, 1) < data.t_s);
/// // The curve has to go down
/// assert!(torque_dyn(&data, Omega(20.0), 12.0, 1) < torque_dyn(&data, Omega(10.0), 12.0, 1));
/// // The curve has to be symmetrical
/// assert_eq!(torque_dyn(&data, Omega(10.0), 12.0, 1), torque_dyn(&data, Omega(10.0), 12.0, 1));
/// ```
pub fn torque_dyn(consts : &StepperConst, mut omega : Omega, u : f32) -> Force {
    omega = omega.abs();

    if !omega.is_finite() {
        panic!("Bad omega! {}", omega);
    }
    
    if omega == Omega::ZERO {
        return consts.t_s;
    }

    let t = consts.full_step_time(omega);
    let pow = E.powf( -t / consts.tau(u) );

    (1.0 - pow) / (1.0 + pow) * consts.t_s
}

/// An approximate torque function
pub fn torque_dyn_approx(consts : &StepperConst, mut omega : Omega, max_omega : Omega) -> Force {
    omega = omega.abs();

    if !omega.is_finite() {
        panic!("Bad omega! {}", omega);
    }

    if omega > max_omega {
        return Force::ZERO;
    }
    
    consts.t_s * (1.0 - omega / max_omega)
}

/// Calculates all forces and torques acting upon a segment connected with another support segment
/// 
/// - `actors` is a tuple vector of all forces acting upon the segment, each tuple consists of (`f_a`, `a_f`)
///     - `f_a` being the actual force acting
///     - `a_f` being the positional vector of the force
/// - `torque` is the torque acting on the center point
/// - `a_s` is the position of the segment support that creates the reaction force 
/// - `a_hat` is the direction of this support
/// 
/// Returns the two resulting forces (`f_c`, `f_j`)
/// - `f_s` being the force acting on the support segment 
/// - `f_j` being the force acting on the joint (center point, position reference point)
/// 
/// ```rust
/// use syact::math::force::forces_segment;
/// 
/// use glam::Vec3;
/// 
/// // Force acting upon the end of a 2 meter rod
/// let actors = vec![
///     // Force of 50 Newton in negative Z direction (downwards); Rod length of 2 meters in positive Y direction 
///     ( Vec3::NEG_Z * 50.0, Vec3::Y * 2.0 )   
/// ];
/// 
/// let ( f_support, f_bearing ) = forces_segment(&actors, 
///     Vec3::ZERO,         // No torque acting upon the bearing
///     Vec3::Y * 1.0,      // Segment support is mounted in the center of the rod
///     Vec3::NEG_Z         // Support is facing downwards
/// );
/// 
/// // Reaction forces
/// assert_eq!(f_support, Vec3::NEG_Z * 100.0);    
/// assert_eq!(f_bearing, Vec3::Z * 50.0);          
/// ```
pub fn forces_segment(actors : &Vec<(Vec3, Vec3)>, mut torque : Vec3, a_s : Vec3, a_hat : Vec3) -> (Vec3, Vec3) {
    let eta_ = a_s.cross(a_hat);
    let eta = eta_ / eta_.length().powi(2);
    
    let mut f_j = Vec3::ZERO;

    for (f_a, a_f) in actors {
        torque += a_f.cross(*f_a);
        f_j += *f_a;
    }

    let f_s = torque.dot(eta) * a_hat;

    (f_s, f_j - f_s) 
}

/// Calculates all the forces in a joint without any support 
/// 
/// - `actors` is a tuple vector of all forces acting upon the segment, each tuple consists of (`f_a`, `a_f`)
///     - `f_a` being the actual force acting
///     - `a_f` being the positional vector of the force
/// - `torque` is the torque acting on the center point
/// 
/// Returns the resulting torque and the resulting joint force (`torque`, `f_j`)
/// 
/// ```rust
/// use syact::math::force::forces_joint;
/// 
/// use glam::Vec3;
/// 
/// // Force acting upon the end of a 2 meter rod
/// let actors = vec![
///     // Force of 50 Newton in negative Z direction (downwards); Rod length of 2 meters in positive Y direction 
///     ( Vec3::NEG_Z * 50.0, Vec3::Y * 2.0 )   
/// ];
/// 
/// let ( torque, force ) = forces_joint(&actors, 
///     Vec3::ZERO,         // No torque acting upon the bearing
/// );
/// 
/// // Reaction forces and torques
/// assert_eq!(torque, Vec3::NEG_X * 100.0);    
/// assert_eq!(force, Vec3::NEG_Z * 50.0);          
/// ```
pub fn forces_joint(actors : &Vec<(Vec3, Vec3)>, mut torque : Vec3) -> (Vec3, Vec3) {
    let mut f_j = Vec3::ZERO;

    for (f_a, a_f) in actors {
        torque += a_f.cross(*f_a);
        f_j += *f_a; 
    }

    (torque, f_j)
}