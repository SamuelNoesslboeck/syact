#![allow(missing_docs)]

use crate::InterruptReason;
use crate::data::ActuatorState;
use crate::intr::InterruptHandler;
use crate::{ActuatorError, AdvSyncActuator, Interruptible, SyncActuator};
use crate::data::ActuatorVars;
use crate::units::*;

const DEMO_ACT_MAX_SPEED : RadPerSecond = RadPerSecond(50.0);

/// A demo actuator that does no actual movements but simulates the behaviour of an actuator structure
pub struct DemoActuator {
    pub intr_handler : InterruptHandler<Self>,
    vars : ActuatorVars<Rotary>
}

impl DemoActuator {
    pub fn new() -> Self {
        Self {
            intr_handler: InterruptHandler::empty(),

            vars: { 
                let mut v = ActuatorVars::default();
                // We know these values are always going to be zero as the DemoActuator does all movements instantly
                v.velocity = Some(RadPerSecond::ZERO);
                v.acceleration = Some(RadPerSecond2::ZERO);
                v
            }
        }
    }
}

impl SyncActuator for DemoActuator {
    type Units = Rotary;

    /* Variable accessors */
        fn __acc_vars<F, R>(&self, f : F) -> R
            where
                F : FnOnce(&ActuatorVars<Rotary>) -> R
        {
            f(&self.vars)
        }

        fn __acc_vars_mut<F, R>(&mut self, f : F) -> R
        where
            F : FnOnce(&mut ActuatorVars<Rotary>) -> R
        {
            f(&mut self.vars)
        }
    /**/

    /* State */
        fn __vel_possible(&self) -> RadPerSecond {
            DEMO_ACT_MAX_SPEED
        }
    /**/
    
    fn __drive_rel_core(&mut self, rel_dist : <Rotary as UnitSet>::Distance, _speed : Factor) -> Result<(), ActuatorError<Rotary>> {
        let dir = rel_dist.get_direction();

        // Implementation of state update unneccessary, just for showcase here (DemoActuator does movements instantly)
        self.vars.state = ActuatorState::Moving(dir);
        self.vars.velocity = Some(RadPerSecond::ZERO);
        self.vars.acceleration = Some(RadPerSecond2::ZERO);

        let steps = (rel_dist.abs() / 0.1).0.ceil();

        for _ in 0 .. steps as usize {
            self.vars.position += rel_dist / steps;
            
            if self.intr_handler.check_all(self, dir)?.is_some() {
                break;
            }
        }

        self.vars.state = ActuatorState::Idle;
        
        Ok(())
    }
    
    fn __drive_cont_core(&mut self, speed : Factor, dir : Direction) -> Result<(), ActuatorError<Rotary>> {
        self.vars.state = ActuatorState::Moving(dir);
        self.vars.velocity = Some(self.vel_possible() * speed);
        self.vars.acceleration = Some(RadPerSecond2::ZERO);
        Ok(())
    }
}

impl Interruptible for DemoActuator {
    fn fetch_intr_reason(&self) -> Option<InterruptReason> {
        self.intr_handler.fetch_intr_reason()
    }
}

impl AdvSyncActuator for DemoActuator {
    fn update_load_calc(&mut self) -> Result<(), ActuatorError<Self::Units>> {
        Ok(())      // Nothing has to be updated
    }
}