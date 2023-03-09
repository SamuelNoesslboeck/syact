use core::f32::consts::{E, PI};

use crate::data::StepperConst;
use crate::units::*;

/// Returns the current torque [Force] that a DC-Motor can produce when driving with the speed `omega` and the voltage `u` in Volts
pub fn torque_dyn(data : &StepperConst, mut omega : Omega, u : f32) -> Force {
    omega = omega.abs();
    
    if omega == Omega::ZERO {
        return data.t_s;
    }

    let t = 2.0 * PI / (data.n_c as f32) / omega;
    let pow = E.powf( -t / data.tau(u) );

    (1.0 - pow) / (1.0 + pow) * data.t_s
}