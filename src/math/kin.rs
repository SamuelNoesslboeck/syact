use syunit::*;

pub const ROOT_EPSILON : f32 = -1.0e-4;

pub mod time {
    use super::*;

    pub fn from_velocity_start_end(velocity_0 : Velocity, velocity  : Velocity, delta : Delta) -> Time {
        2.0 * delta / (velocity_0 + velocity )
    }

    #[inline]
    pub fn positive_travel_time(delta : Delta, velocity  : Velocity, alpha : Acceleration) -> Option<Time> {
        let (mut t_1, mut t_2) = travel_times(delta, velocity , alpha);

        if t_1.is_sign_negative() {
            t_1 = Time::INFINITY;       // Invaild time

            if t_2.is_sign_negative() {
                return None;            // Both invalid, no time can be determined
            }
        } else if t_2.is_sign_negative() {
            t_2 = Time::INFINITY;       // Invalid time
        }

        Some(t_1.min(t_2))
    }
}

pub mod delta {
    use super::*;

    #[inline]
    pub fn from_velocity_start_end(velocity_0 : Velocity, velocity  : Velocity, time : Time) -> Delta {
        (velocity_0 + velocity ) / 2.0 * time
    }

    #[inline]
    pub fn from_velocity_alpha(velocity_0 : Velocity, alpha : Acceleration, time : Time) -> Delta {
        velocity_0 * time + alpha * time * time / 2.0
    }
}

pub mod velocity {
    use super::*;

    pub fn from_delta_time_0(velocity_0 : Velocity, delta : Delta, time : Time) -> Velocity {
        2.0 * delta / time - velocity_0
    }
}

/// Calculates the two possible travel times for a physical object 
/// 
/// # Panics 
/// 
/// The function panics if the given alpha is not normal (`Acceleration::is_normal()`)
#[inline]
pub fn travel_times(delta : Delta, velocity : Velocity, alpha : Acceleration) -> (Time, Time) {
    if !alpha.is_normal() {
        panic!("The given alpha is invalid (delta: {}, velocity : {}, alpha: {})", delta, velocity , alpha);
    }

    let p = velocity  / alpha;
    let q = 2.0 * delta.0 / alpha.0; 

    let inner = p.0.powi(2) + q;
    let mut root = Time(inner.sqrt());

    if inner.is_sign_negative() & (inner > ROOT_EPSILON) {
        root = Time::ZERO;
    }

    ( -p + root, -p - root )
}

pub fn accel_from_zero(delta : Delta, alpha : Acceleration) -> Time {
    Time((2.0 * delta.0 / alpha.0).sqrt())
}

pub fn alpha_req_for_dist(delta : Delta, velocity  : Velocity) -> Acceleration {
    Acceleration(velocity .0 * velocity .0 / 2.0 / delta.0)
}

// Stepper
pub fn velocity_start_stop(force_stall : Force, inertia : Inertia, number_of_steps : u64) -> Velocity {
    Velocity((force_stall.0 / inertia.0 * core::f32::consts::PI / number_of_steps as f32).sqrt())
}