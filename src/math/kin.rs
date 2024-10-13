use syunit::*;

/// Defined inaccuracy
pub const ROOT_EPSILON : f32 = -1.0e-4;

/// Calculates the two possible travel times for a physical object 
/// 
/// # Panics 
/// 
/// The function panics if the given acceleration is not normal (`Acceleration::is_normal()`)
#[inline]
pub fn travel_times(rel_pos : RelDist, velocity : Velocity, acceleration : Acceleration) -> (Time, Time) {
    if !acceleration.is_normal() {
        panic!("The given acceleration is invalid (rel_pos: {}, velocity : {}, acceleration: {})", rel_pos, velocity, acceleration);
    }

    let p = velocity / acceleration;
    let q = 2.0 * rel_pos.0 / acceleration.0; 

    let inner = p.0.powi(2) + q;
    let mut root = Time(inner.sqrt());

    if inner.is_sign_negative() & (inner > ROOT_EPSILON) {
        root = Time::ZERO;
    }

    ( -p + root, -p - root )
}

/// Time required to move a distance `rel_pos` from a zero-velocity state with the acceleration `acceleration`
pub fn accel_from_zero(rel_pos : RelDist, acceleration : Acceleration) -> Time {
    Time((2.0 * rel_pos.0 / acceleration.0).sqrt())
}

/// The acceleration required to exit a certain distance `rel_pos` with the given velocity `velocity`, starting with a velocity of zero
pub fn alpha_req_for_dist(rel_pos : RelDist, velocity : Velocity) -> Acceleration {
    Acceleration(velocity.0 * velocity.0 / 2.0 / rel_pos.0)
}

// Stepper
/// Returns the start-stop-velocity for a stepper motor
pub fn velocity_start_stop(torque_stall : Force, inertia : Inertia, number_of_steps : u64) -> Velocity {
    Velocity((torque_stall.0 / inertia.0 * core::f32::consts::PI / number_of_steps as f32).sqrt())
}