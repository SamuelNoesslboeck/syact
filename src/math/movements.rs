use alloc::vec::Vec;

use syunit::*;

use crate::{SyncActuator, SyncActuatorGroup};

/// An actuator that has a defined time to move for a PTP (Point-To-Point) movement
pub trait DefinedActuator {
    /// The time required to perform a certain PTP (Point-To-Point movement)
    fn ptp_time_for_distance(&self, abs_pos_0 : AbsPos, abs_pos_t : AbsPos) -> Time;
}

/// More calculation intense, no additional memory
pub fn ptp_speed_factors<S : SyncActuatorGroup<T, C>, T : SyncActuator + DefinedActuator + ?Sized + 'static, const C : usize>
    (group : &mut S, abs_pos_0 : [AbsPos; C], abs_pos_t : [AbsPos; C], speed : Factor) -> [Factor; C] 
{
    let times = group.for_each(|comp, index| {
        comp.ptp_time_for_distance(abs_pos_0[index], abs_pos_t[index])
    });

    // Safe to unwrap, cause iterator is not empty
    let time_max = *times.iter().reduce(Time::max_ref).unwrap();

    times.iter().map(|time| Factor::try_new(*time / time_max).unwrap_or(Factor::MAX) * speed)
        .collect::<Vec<Factor>>().try_into().unwrap()
}