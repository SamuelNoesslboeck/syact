use core::sync::atomic::AtomicBool;

use crate::Setup;
use crate::ctrl::{Interruptor, InterruptReason};
use crate::ctrl::pin::{UniInPin, UniPin};

use alloc::sync::Arc;
use atomic_float::AtomicF32;
use serde::{Serialize, Deserialize};

use super::*;

/// A simple endswitch that can trigger when reaching a destination
#[cfg_attr(feature = "std", derive(Serialize, Deserialize))]
pub struct EndSwitch {
    _data : SimpleMeasData,
    pin : u8,

    #[cfg_attr(feature = "std", serde(skip))]
    sys_pin : Option<UniInPin>
}

impl EndSwitch {
    /// Creates a new end switch
    pub fn new(pin : u8, data : SimpleMeasData) -> Self {
        Self {
            _data: data,
            pin, 
            sys_pin: None
        }
    }
}

impl Setup for EndSwitch {
    fn setup(&mut self) -> Result<(), crate::Error> {
        self.sys_pin = Some(UniPin::new(self.pin)?.into_input());
        Ok(())
    }
}

impl Interruptor for EndSwitch {
    fn check(&mut self, _gamma : &Arc<AtomicF32>) -> Option<InterruptReason> {
        if let Some(pin) = &mut self.sys_pin {
            if pin.is_high() {
                Some(InterruptReason::EndReached)
            } else {
                None
            }
        } else {
            Some(InterruptReason::Error) // TODO: add error
        }
    }
}

impl SimpleMeas for EndSwitch {
    fn data(&self) -> &SimpleMeasData {
        &self._data
    }
}

// Virtual
pub struct VirtualEndSwitch {
    pub vpin : Arc<AtomicBool>
}

impl VirtualEndSwitch {
    pub fn new(def : bool) -> Self {
        VirtualEndSwitch { 
            vpin: Arc::new(AtomicBool::new(def))
        }
    }
}

impl Setup for VirtualEndSwitch { }

impl Interruptor for VirtualEndSwitch {
    fn check(&mut self, _gamma : &Arc<AtomicF32>) -> Option<InterruptReason> {
        if self.vpin.load(core::sync::atomic::Ordering::Relaxed) {
            Some(InterruptReason::EndReached)
        } else {
            None
        }
    }
}
