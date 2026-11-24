#![allow(missing_docs)]

use syunit::metric::KgMeter2;
use syunit::metric::NewtonMeters;

use crate::InterruptReason;
use crate::intr::InterruptHandler;
use crate::{ActuatorError, AdvSyncActuator, Interruptible, SyncActuator};
use crate::data::ActuatorVars;
use crate::units::*;

const DEMO_ACT_MAX_SPEED : RadPerSecond = RadPerSecond(50.0);

/// A demo actuator that does no actual movements but simulates the behaviour of an actuator structure
pub struct DemoActuator {
    _pos : PositionRad,
    _vel : RadPerSecond,
    _dir : Direction,

    pub intr_handler : InterruptHandler<Self>,

    vars : ActuatorVars
}

impl DemoActuator {
    pub fn new() -> Self {
        Self {
            _pos: PositionRad::ZERO,
            _vel: RadPerSecond::ZERO,
            _dir: Direction::default(),

            intr_handler: InterruptHandler::new(),

            vars: ActuatorVars::default()
        }
    }
}

impl SyncActuator for DemoActuator {
    type Units = Rotary;

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

    /* Position */
        fn pos(&self) -> PositionRad {
            self._pos
        }

        fn overwrite_abs_pos(&mut self, pos : PositionRad) {
            self._pos = pos;
        }
    /**/

    /* State */
        fn direction(&self) -> Direction {
            self._dir
        }

        fn __vel_possible(&self) -> RadPerSecond {
            DEMO_ACT_MAX_SPEED
        }

        fn velocity(&self) -> RadPerSecond {
            self._vel
        }
        
        fn is_moving(&self) -> bool {
            false       // Demo actuator does all movements instantly
        }
    /**/
    
    fn __drive_rel_core(&mut self, rel_dist : <Rotary as UnitSet>::Distance, _speed : Factor) -> Result<(), ActuatorError<Rotary>> {
        self._dir = rel_dist.get_direction();
        self._vel = RadPerSecond::ZERO;

        let steps = (rel_dist.abs() / 0.1).0.ceil();

        for _ in 0 .. steps as usize {
            self._pos += rel_dist / steps;
            
            if self.intr_handler.check_all(self)?.is_some() {
                break;
            }
        }
        
        Ok(())
    }
    
    fn __drive_cont_core(&mut self, speed : Factor, direction : Direction) -> Result<(), ActuatorError<Rotary>> {
        self._vel = DEMO_ACT_MAX_SPEED * speed;
        self._dir = direction;
        Ok(())
    }
}

impl Interruptible for DemoActuator {
    fn fetch_intr_reason(&self) -> Option<InterruptReason> {
        self.intr_handler.fetch_intr_reason()
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