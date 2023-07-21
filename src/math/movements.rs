use crate::comp::stepper::{StepperComp, StepperCompGroup};
use crate::math::HRLimitedStepBuilder;
use crate::units::*;

/// More calculation intense, no additional memory
pub fn ptp_exact_unbuffered<S : StepperCompGroup<T, C>, T : StepperComp + ?Sized + 'static, const C : usize>(group : &mut S, deltas : [Delta; C]) -> [f32; C] {
    let mut times : [f32; C] = group.for_each(|p_comp, index| {
        // Curves are built for motors
        let comp = p_comp.motor();

        let mut builder = HRLimitedStepBuilder::from_builder(
            comp.create_hr_builder(Omega::ZERO, comp.omega_max())
        );

        builder.set_omega_tar(comp.omega_max()).unwrap();   // Save to unwrap
        builder.set_steps_max(comp.consts().steps_from_ang_abs(deltas[index], comp.micro()));
        
        builder.map(|t| t.0).sum()
    });

    let max = times.into_iter().reduce(f32::max).unwrap();

    for i in 0 .. C {
        times[i] = times[i] / max;
    }

    times
}