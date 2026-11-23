#![allow(missing_docs)]

use alloc::boxed::Box;
use alloc::vec::Vec;
use syunit::metric::KgMeter2;
use syunit::metric::NewtonMeters;

use crate::InterruptReason;
use crate::{ActuatorError, AdvSyncActuator, Interruptible, SyncActuator};
use crate::data::ActuatorVars;
use crate::intr::Interruptor;
use crate::units::*;

const DEMO_ACT_MAX_SPEED : RadPerSecond = RadPerSecond(50.0);

/// A demo actuator that does no actual movements but simulates the behaviour of an actuator structure
pub struct DemoActuator {
    _pos : PositionRad,
    _vel : RadPerSecond,
    _dir : Direction,

    intr : Vec<Box<dyn Interruptor>>,
    intr_reason : Option<InterruptReason>,

    vars : ActuatorVars
}

impl DemoActuator {
    pub fn new() -> Self {
        Self {
            _pos: PositionRad::ZERO,
            _vel: RadPerSecond::ZERO,
            _dir: Direction::default(),

            intr: Vec::new(),
            intr_reason: None,

            vars: ActuatorVars::default()
        }
    }
}

impl SyncActuator<Rotary> for DemoActuator {
    /* Variable accessors */
        fn acc_vars<F, R>(&self, f : F) -> R
            where
                F : FnOnce(&ActuatorVars<Rotary>) -> R
        {
            f(&self.vars)
        }

        fn acc_vars_mut<F, R>(&mut self, f : F) -> R
        where
            F : FnOnce(&mut ActuatorVars<Rotary>) -> R
        {
            f(&mut self.vars)
        }
    /**/

    /* State */
        fn direction(&self) -> Direction {
            self._dir
        }

        fn velocity(&self) -> RadPerSecond {
            self._vel
        }
        
        fn is_moving(&self) -> bool {
            false       // Demo actuator does all movements instantly
        }
    /**/


    fn pos(&self) -> PositionRad {
        self._pos
    }

    fn overwrite_abs_pos(&mut self, pos : PositionRad) {
        self._pos = pos;
    }
    
    fn drive_rel(&mut self, rel_dist : <Rotary as UnitSet>::Distance, _speed : Factor) -> Result<(), ActuatorError<Rotary>> {
        self._dir = rel_dist.get_direction();
        self._vel = RadPerSecond::ZERO;

        let steps = (rel_dist.abs() / 0.1).0.ceil();

        for i in 0 .. steps as usize {
            self._pos += rel_dist / steps;
            
            for intr in self.intr {
                let reason = intr.check();
                if reason.is_some() {
                    self.intr_reason = reason;
                    break;
                }
            }
        }
        
        Ok(())
    }
    
    fn drive_cont_factor(&mut self, speed : Factor, direction : Direction) -> Result<(), ActuatorError<Rotary>> {
        self._vel = DEMO_ACT_MAX_SPEED * speed;
        self._dir = direction;
        Ok(())
    }
    
    fn drive_cont_speed(&mut self, speed : <Rotary as UnitSet>::Velocity) -> Result<(), ActuatorError<Rotary>> {
        if speed.abs() > DEMO_ACT_MAX_SPEED {
            return Err(ActuatorError::VelocityTooHigh(speed, DEMO_ACT_MAX_SPEED));
        }

        self._dir = speed.get_direction();
        self._vel = speed.abs();
        Ok(())
    }
}

impl Interruptible for DemoActuator {
    fn add_interruptor(&mut self, interruptor : alloc::boxed::Box<dyn Interruptor + Send>) {
        self.intr.push(interruptor);
    }

    fn fetch_intr_reason(&mut self) -> Option<InterruptReason> {
        self.intr_reason.take()   
    }
}

impl AdvSyncActuator for DemoActuator {
    fn apply_gen_force(&mut self, force : NewtonMeters) -> Result<(), ActuatorError<Rotary>> {
        self.vars.load_force_gen = force;   // TODO: Maybe add overloads?
        Ok(())
    }
    
    fn apply_dir_force(&mut self, force : NewtonMeters) -> Result<(), ActuatorError<Rotary>> {
        self.vars.load_force_dir = force;
        Ok(())
    }
    
    fn apply_load_inertia(&mut self, inertia : KgMeter2) -> Result<(), ActuatorError<Rotary>>  {
        self.vars.load_inertia = inertia;
        Ok(())
    }
}