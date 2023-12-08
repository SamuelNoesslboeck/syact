use core::sync::atomic::AtomicBool;

use alloc::sync::Arc;
use atomic_float::AtomicF32;
use embedded_hal::digital::v2::InputPin;
use serde::{Serialize, Deserialize};

use crate::Setup;
use crate::act::{Interruptor, InterruptReason};
use crate::device::pin::UniInPin;

use super::*;

/// A simple endswitch that can trigger when reaching a destination
#[derive(Serialize, Deserialize)]
pub struct RawEndSwitch<P : InputPin> {
    trigger : bool,
    _dir : Option<sylo::Direction>, 
    temp_dir : Option<sylo::Direction>,

    #[serde(skip)]
    sys_pin : P
}

impl<P : InputPin> RawEndSwitch<P> {
    /// Creates a new end switch
    pub fn new(trigger : bool, dir : Option<sylo::Direction>, sys_pin : P) -> Self {
        Self {
            trigger,
            _dir: dir,
            temp_dir: None,

            sys_pin
        }
    }
}

impl<P : InputPin> BoolMeas for RawEndSwitch<P> {
    type Error = (); 

    fn meas(&mut self) -> Result<bool, Self::Error> {
        return Ok(unsafe { self.sys_pin.is_high().unwrap_unchecked() } == self.trigger)
    }
}

impl<P : InputPin> Interruptor for RawEndSwitch<P> {
    fn dir(&self) -> Option<sylo::Direction> {
        self._dir.or(self.temp_dir)
    }

    fn set_temp_dir(&mut self, dir_opt : Option<sylo::Direction>) {
        self.temp_dir = dir_opt;
    }

    fn check(&mut self, _gamma : &Arc<AtomicF32>) -> Option<InterruptReason> {
        // unwraping unsafe is safe, as no error can occur
        if unsafe { self.sys_pin.is_high().unwrap_unchecked() } == self.trigger {    
            Some(InterruptReason::EndReached)
        } else {
            None
        }
    }
}

// Virtual
pub struct VirtualEndSwitch {
    pub vpin : Arc<AtomicBool>,
    _dir : Option<sylo::Direction>,
    temp_dir : Option<sylo::Direction>
}

impl VirtualEndSwitch {
    pub fn new(def : bool, _dir : Option<sylo::Direction>) -> Self {
        VirtualEndSwitch { 
            vpin: Arc::new(AtomicBool::new(def)),
            _dir,
            temp_dir: None
        }
    }
}

impl Setup for VirtualEndSwitch { }

impl Interruptor for VirtualEndSwitch {
    fn dir(&self) -> Option<sylo::Direction> {
        self._dir.or(self.temp_dir)
    }

    fn set_temp_dir(&mut self, dir_opt : Option<sylo::Direction>) {
        self.temp_dir = dir_opt;
    } 
    
    fn check(&mut self, _gamma : &Arc<AtomicF32>) -> Option<InterruptReason> {
        if self.vpin.load(core::sync::atomic::Ordering::Relaxed) {
            Some(InterruptReason::EndReached)
        } else {
            None
        }
    }
}

// #########################
// #    IMPLEMENTATIONS    #
// #########################
    // `UniPin` - Implementation
    pub type EndSwitch = RawEndSwitch<UniInPin>;

    impl<P : InputPin + Setup> Setup for RawEndSwitch<P> {
        fn setup(&mut self) -> Result<(), crate::Error> {
            self.sys_pin.setup()
        }
    } 
//