use alloc::vec;
use alloc::vec::Vec;

use core::f32::consts::PI;

use crate::data::{StepperConst, CompVars, LinkedData};
use crate::units::*;

use crate::math::CurveBuilder;

/// Returns the start freqency of a motor (data)  \
/// 
/// # Units
/// 
///  - Returns Hertz
#[inline]
pub fn start_frequency(data : &StepperConst, var : &CompVars) -> Omega {
    Omega((data.alpha_max(var).unwrap() * (data.n_s as f32) / 4.0 / PI).0.powf(0.5))        // TODO: Overload
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
    let root = if inner.abs() > f32::EPSILON {
        Time(inner.sqrt())
    } else {
        Time::ZERO
    };

    ( -p + root, -p - root )
}

/// Calculates the next node in a curve
/// 
/// # Panics 
/// 
/// The function panics if the given alpha is not normal (`Alpha::is_normal()`)
#[inline]
pub fn next_node_simple(delta : Delta, omega_0 : Omega, alpha : Alpha) -> (Time, Omega) {
    let (mut t_1, mut t_2) = travel_times(delta, omega_0, alpha);

    if t_1 < Time::ZERO {
        t_1 = Time::INFINITY;
    }

    if t_2 < Time::ZERO {
        t_2 = Time::INFINITY;
    }

    let t = t_1.min(t_2);

    ( t, omega_0 + alpha * t )
}

// Curves
/// Creates a new empty curve 
#[inline]
pub fn crate_plain_curve(data : &StepperConst, node_count : usize, omega_max : Omega) -> Vec<Time> {
    vec![data.step_time(omega_max); node_count]
}

/// Mirrors the curve in the center, existing values will be overwritten
pub fn mirror_curve(cur : &mut [Time]) {
    let cur_len = cur.len();

    for i in 0 .. cur_len / 2 {
        cur[cur_len - i - 1] = cur[i];
    }
}

/// Writes a simple stepper motor acceleration curve to a given curve `cur` 
/// 
/// # Panics
/// 
/// Panics if the given bend or safety factor is invalid
pub fn write_simple_move(consts : &StepperConst, var : &CompVars, lk : &LinkedData, cur : &mut [Time], omega_max : Omega) {
    let cur_len = cur.len(); 

    let mut builder = CurveBuilder::new(consts, var, lk, Omega::ZERO);

    for i in 0 .. cur_len / 2 {
        let time = builder.next_step_pos();

        if builder.omega > omega_max {
            break;
        }

        cur[i] = time;
    }

    mirror_curve(cur);
}

/// Create a new simple acceleration curve that can be used by a stepper motor to drive safely from standstill
#[inline]
pub fn create_simple_curve(consts : &StepperConst, vars : &CompVars, lk : &LinkedData, delta : Delta, omega_max : Omega) 
        -> Vec<Time> {
    let steps = consts.steps_from_ang_abs(delta);

    let mut cur = crate_plain_curve(consts, steps as usize, omega_max);
    write_simple_move(consts, vars, lk, cur.as_mut_slice(), omega_max);

    cur
}