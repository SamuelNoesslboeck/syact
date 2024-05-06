use crate::act::stepper::{StepperActuator, StepperActuatorGroup};
use syunit::*;

pub trait DefinedActuator {
    fn ptp_time_for_distance(&self, gamma_0 : Gamma, gamma_t : Gamma) -> Time;
}

/// More calculation intense, no additional memory
pub fn ptp_speed_factors<S : StepperActuatorGroup<T, C>, T : StepperActuator + DefinedActuator + ?Sized + 'static, const C : usize>
    (group : &mut S, gamma_0 : [Gamma; C], gamma_t : [Gamma; C], speed : Factor) -> [Factor; C] 
{
    let times = group.for_each(|comp, index| {
        comp.ptp_time_for_distance(gamma_0[index], gamma_t[index])
    });

    // Safe to unwrap, cause iterator is not empty
    let time_max = *times.iter().reduce(Time::max_ref).unwrap();

    times.iter().map(|time| Factor::try_new(*time / time_max).unwrap_or(Factor::MAX) * speed)
        .collect::<Vec<Factor>>().try_into().unwrap()
}