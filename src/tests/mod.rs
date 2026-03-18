#![allow(missing_docs)]

use syunit::metric::KgMeter2;
use syunit::metric::NewtonMeters;

use crate::ActuatorError;
use crate::AdvancedActuator;
use crate::SyncActuator;
use crate::data::ActuatorVars;
use crate::units::*;

/// Testing
#[derive(Clone, Debug, Default)]
pub struct DemoActuator {
    _pos : PositionRad,

    _velocity_max : Option<RadPerSecond>,
    _acceleration_max : Option<RadPerSecond2>,
    _jolt_max : Option<RadPerSecond3>,

    _limit_min : Option<PositionRad>,
    _limit_max : Option<PositionRad>,

    dir : Direction,

    vars : ActuatorVars
}

impl DemoActuator {
    pub fn new() -> Self {
        Self::default()
    }
}

impl SyncActuator<Rotary> for DemoActuator {
    fn pos(&self) -> PositionRad {
        self._pos
    }

    fn overwrite_abs_pos(&mut self, pos : PositionRad) {
        self._pos = pos;
    }

    fn velocity_max(&self) -> Option<RadPerSecond> {
        self._velocity_max
    }

    fn set_velocity_max(&mut self, velocity_opt : Option<RadPerSecond>) -> Result<(), ActuatorError<Rotary>> {
        self._velocity_max = velocity_opt;
        Ok(())
    }

    fn acceleration_max(&self) -> Option<RadPerSecond2> {
        self._acceleration_max
    }

    fn set_acceleration_max(&mut self, acceleration_opt : Option<RadPerSecond2>) -> Result<(), ActuatorError<Rotary>> {
        self._acceleration_max = acceleration_opt;
        Ok(())
    }

    fn jolt_max(&self) -> Option<RadPerSecond3> {
        self._jolt_max
    }

    fn set_jolt_max(&mut self, jolt_opt : Option<RadPerSecond3>) -> Result<(), ActuatorError<Rotary>> {
        self._jolt_max = jolt_opt;
        Ok(())
    }

    fn limit_min(&self) -> Option<PositionRad> {
        self._limit_min
    }

    fn limit_max(&self) -> Option<PositionRad> {
        self._limit_max
    }

    fn set_endpos(&mut self, overwrite_abs_pos : PositionRad) {
        self.overwrite_abs_pos(overwrite_abs_pos);

        let dir = self.dir.as_bool();

        self.set_pos_limits(
            if dir { None } else { Some(overwrite_abs_pos) },
            if dir { Some(overwrite_abs_pos) } else { None }
        )
    }

    fn set_pos_limits(&mut self, min : Option<PositionRad>, max : Option<PositionRad>) {
        if let Some(min) = min {
            self._limit_min = Some(min)
        }

        if let Some(max) = max {
            self._limit_max = Some(max);
        }
    }

    fn overwrite_pos_limits(&mut self, min : Option<PositionRad>, max : Option<PositionRad>) {
        self._limit_min = min;
        self._limit_max = max;
    }
    
    fn clone_state(&self) -> alloc::sync::Arc<dyn crate::SyncActuatorState<Rotary>> {
        todo!()
    }
    
    async fn drive_rel(&mut self, rel_dist : <Rotary as UnitSet>::Distance, speed : Factor) -> Result<(), ActuatorError<Rotary>> {
        todo!()
    }
    
    async fn drive_factor(&mut self, speed : Factor, direction : Direction) -> Result<(), ActuatorError<Rotary>> {
        todo!()
    }
    
    async fn drive_speed(&mut self, speed : <Rotary as UnitSet>::Velocity) -> Result<(), ActuatorError<Rotary>> {
        todo!()
    }
}

impl AdvancedActuator for DemoActuator {
    fn force_gen(&self) -> <Rotary as UnitSet>::Force {
        self.vars.force_load_gen
    }
    
    fn force_dir(&self) -> <Rotary as UnitSet>::Force {
        self.vars.force_load_dir
    }
    
    fn apply_gen_force(&mut self, force : NewtonMeters) -> Result<(), ActuatorError<Rotary>> {
        self.vars.force_load_gen = force;
        Ok(())
    }
    
    fn apply_dir_force(&mut self, force : NewtonMeters) -> Result<(), ActuatorError<Rotary>> {
        self.vars.force_load_dir = force;
        Ok(())
    }
    
    fn inertia(&self) -> KgMeter2 {
        self.vars.inertia_load
    }
    
    fn apply_inertia(&mut self, inertia : KgMeter2) -> Result<(), ActuatorError<Rotary>>  {
        self.vars.inertia_load = inertia;
        Ok(())
    }
}