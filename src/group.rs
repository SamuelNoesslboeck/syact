use syunit::*;

use crate::ActuatorError;

pub trait SyncGroup<U : UnitSet, const C : usize> {
    fn pos(&self) -> [U::Position; C];

    async fn drive_rel(&mut self, rel_dist : [U::Distance; C], speed : Factor) -> ActuatorError<U>;

    async fn drive_abs(&mut self, pos : [U::Position; C], speed : Factor) -> ActuatorError<U>;
}