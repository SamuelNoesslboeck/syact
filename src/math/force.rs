use core::f32::consts::E;

use syunit::*;

use crate::StepperConst;

/// Returns the current torque [Force] that a DC-Motor can produce when driving with the 
/// speed `omega` and the voltage `u` in Volts
/// 
/// # Panics
/// 
/// Panics if the given omega is not finite
pub fn torque_dyn(consts : &StepperConst, mut omega : Velocity, voltage : f32, overload_current : Option<f32>) -> Force {
    omega = omega.abs();

    if !omega.is_finite() {
        panic!("Bad omega! {}", omega);
    }
    
    if omega == Velocity::ZERO {
        return consts.torque_overload(overload_current);
    }

    let time = consts.full_step_time(omega);
    let pow = E.powf( -time / consts.tau() );

    let t_ol = consts.torque_overload(overload_current);
    let t = (1.0 - pow) / (1.0 + pow) * consts.torque_overload_max(voltage);
    
    if t > t_ol { t_ol } else { t }
}

/// An approximate torque function
pub fn torque_dyn_approx(consts : &StepperConst, mut omega : Velocity, max_omega : Velocity) -> Force {
    omega = omega.abs();

    if !omega.is_finite() {
        panic!("Bad omega! {}", omega);
    }

    if omega > max_omega {
        return Force::ZERO;
    }
    
    consts.torque_stall * (1.0 - omega / max_omega)
}