use crate::units::*;

// Submodules
pub mod actors;

pub mod curve;

pub mod force;

pub mod inertia;

pub mod load;

// pub fn write_accel(cur : &mut [Time], consts : &StepperConst, var : &CompVars, omega_0 : Omega, omega_tar : Omega) {

// }

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

    fn accel_max_node(&self, gamma_0 : Gamma, delta_pos : Delta, omega_0 : Omega, omega_max : Omega) -> (Alpha, Alpha) {
        // Get maximum accelerations
        let ( t_pos, mut accel_max_pos ) = self.node_from_vel(delta_pos, omega_0, omega_max.abs());
        let ( t_neg, mut accel_max_neg ) = self.node_from_vel(delta_pos, omega_0, -(omega_max.abs()));

        let accel = self.accel_dyn(((omega_0 + omega_max) / 2.0).abs(), gamma_0);

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