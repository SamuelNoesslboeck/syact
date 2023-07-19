use crate::units::*;

pub const ROOT_EPSILON : f32 = -1.0e-4;

impl Delta {
    #[inline]
    pub fn from_omega_start_end(omega_0 : Omega, omega : Omega, time : Time) -> Delta {
        (omega_0 + omega) / 2.0 * time
    }

    #[inline]
    pub fn from_omega_alpha(omega_0 : Omega, alpha : Alpha, time : Time) -> Delta {
        omega_0 * time + alpha * time * time / 2.0
    }
}

/// Calculates the two possible travel times for a physical object 
/// 
/// # Panics 
/// 
/// The function panics if the given alpha is not normal (`Alpha::is_normal()`)
#[inline]
pub fn travel_times(delta : Delta, omega : Omega, alpha : Alpha) -> (Time, Time) {
    if !alpha.is_normal() {
        panic!("The given alpha is invalid (delta: {}, omega: {}, alpha: {})", delta, omega, alpha);
    }

    let p = omega / alpha;
    let q = 2.0 * delta.0 / alpha.0; 

    let inner = p.0.powi(2) + q;
    let mut root = Time(inner.sqrt());

    if inner.is_sign_negative() & (inner > ROOT_EPSILON) {
        root = Time::ZERO;
    }

    ( -p + root, -p - root )
}

impl Time {
    #[inline]
    pub fn positive_travel_time(delta : Delta, omega : Omega, alpha : Alpha) -> Option<Time> {
        let (mut t_1, mut t_2) = travel_times(delta, omega, alpha);

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