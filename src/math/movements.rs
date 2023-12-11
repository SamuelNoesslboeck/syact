use crate::SpeedFactor;
use crate::act::stepper::{StepperActuator, StepperActuatorGroup};
use crate::math::HRLimitedStepBuilder;
use crate::units::*;

use super::HRStepBuilder;

/// More calculation intense, no additional memory
pub fn ptp_exact_unbuffered<S : StepperActuatorGroup<T, C>, T : StepperActuator + ?Sized + 'static, const C : usize>
    (group : &mut S, deltas : [Delta; C], speed_f : SpeedFactor) -> [SpeedFactor; C] 
{
    // The times store the movement time
    let mut times : [f32; C] = group.for_each(|p_comp, index| {
        // Curves are built for motors
        let comp = p_comp.motor();

        // Create the builder using the Motor
        let mut builder = HRLimitedStepBuilder::from_builder(
            HRStepBuilder::from_motor(comp, Omega::ZERO)
        );

        builder.set_omega_tar(comp.omega_max()).unwrap();   // Save unwraped
        builder.set_steps_max(comp.consts().steps_from_angle_abs(deltas[index], comp.microsteps()));
        
        builder.map(|t| t.0).sum()
    });

    // The maximum movement time
    let max = times.into_iter().reduce(f32::max).unwrap();

    // Each time is divided by the maximum 
    for i in 0 .. C {
        // (New speed factor) times overall speed factor
        times[i] = (times[i] / max) * speed_f;
    }

    times
}