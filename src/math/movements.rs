use alloc::vec::Vec;

use syunit::*;

use crate::{SyncActuator, SyncActuatorGroup};

/// An actuator that has a defined time to move for a PTP (Point-To-Point) movement
pub trait DefinedActuator {
    /// The time required to perform a certain PTP (Point-To-Point movement)
    fn ptp_time_for_distance(&self, gamma_0 : AbsPos, gamma_t : AbsPos) -> Time;
}

/// More calculation intense, no additional memory
pub fn ptp_speed_factors<S : SyncActuatorGroup<T, C>, T : SyncActuator + DefinedActuator + ?Sized + 'static, const C : usize>
    (group : &mut S, gamma_0 : [AbsPos; C], gamma_t : [AbsPos; C], speed : Factor) -> [Factor; C] 
{
    let times = group.for_each(|comp, index| {
        comp.ptp_time_for_distance(gamma_0[index], gamma_t[index])
    });

    // Safe to unwrap, cause iterator is not empty
    let time_max = *times.iter().reduce(Time::max_ref).unwrap();

    times.iter().map(|time| Factor::try_new(*time / time_max).unwrap_or(Factor::MAX) * speed)
        .collect::<Vec<Factor>>().try_into().unwrap()
}