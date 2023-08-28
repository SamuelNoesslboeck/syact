use core::sync::atomic::AtomicBool;

use crate::{Setup, Direction};
use crate::ctrl::{Interruptor, InterruptReason};
use crate::ctrl::pin::{UniInPin, UniPin};

use alloc::sync::Arc;
use atomic_float::AtomicF32;
use serde::{Serialize, Deserialize};

use super::*;

/// A simple endswitch that can trigger when reaching a destination
#[derive(Serialize, Deserialize)]
pub struct EndSwitch {
    pin : u8,
    trigger : bool,
    _dir : Option<Direction>,

    #[serde(skip)]
    sys_pin : Option<UniInPin>
}

impl EndSwitch {
    /// Creates a new end switch
    pub fn new(pin : u8, trigger : bool, _dir : Option<Direction>) -> Self {
        Self {
            pin, 
            trigger,
            _dir,

            sys_pin: None
        }
    }

    pub fn is_triggered(&self) -> bool {
        self.sys_pin.as_ref()
            .map(|pin| pin.is_high())
            .unwrap_or(false) == self.trigger
    }
}

impl Setup for EndSwitch {
    fn setup(&mut self) -> Result<(), crate::Error> {
        self.sys_pin = Some(UniPin::new(self.pin)?.into_input());
        Ok(())
    }
}

impl Interruptor for EndSwitch {
    fn dir(&self) -> Option<Direction> {
        self._dir
    }

    fn check(&mut self, _gamma : &Arc<AtomicF32>) -> Option<InterruptReason> {
        if let Some(pin) = &mut self.sys_pin {
            if pin.is_high() == self.trigger {
                Some(InterruptReason::EndReached)
            } else {
                None
            }
        } else {
            Some(InterruptReason::Error) // TODO: add error
        }
    }
}

impl SimpleMeas for EndSwitch { }

// Virtual
pub struct VirtualEndSwitch {
    pub vpin : Arc<AtomicBool>,
    _dir : Option<Direction>
}

impl VirtualEndSwitch {
    pub fn new(def : bool, _dir : Option<Direction>) -> Self {
        VirtualEndSwitch { 
            vpin: Arc::new(AtomicBool::new(def)),
            _dir
        }
    }
}

impl Setup for VirtualEndSwitch { }

impl Interruptor for VirtualEndSwitch {
    fn dir(&self) -> Option<Direction> {
        self._dir
    }
    
    fn check(&mut self, _gamma : &Arc<AtomicF32>) -> Option<InterruptReason> {
        if self.vpin.load(core::sync::atomic::Ordering::Relaxed) {
            Some(InterruptReason::EndReached)
        } else {
            None
        }
    }
}
