use core::sync::atomic::AtomicBool;

use crate::{Setup, Direction};
use crate::ctrl::{Interruptor, InterruptReason};
use crate::ctrl::pin::{UniPin, UniInPin};

use alloc::sync::Arc;
use atomic_float::AtomicF32;
use embedded_hal::digital::v2::InputPin;
use serde::{Serialize, Deserialize};

use super::*;

/// A simple endswitch that can trigger when reaching a destination
#[derive(Serialize, Deserialize)]
pub struct RawEndSwitch<P : InputPin> {
    trigger : bool,
    _dir : Option<Direction>,       // TODO: Maybe move to interruptor?

    #[serde(skip)]
    sys_pin : P
}

impl<P : InputPin> RawEndSwitch<P> {
    /// Creates a new end switch
    pub fn new(trigger : bool, _dir : Option<Direction>, sys_pin : P) -> Self {
        Self {
            trigger,
            _dir,

            sys_pin
        }
    }
}

impl<P : InputPin> BoolMeas for RawEndSwitch<P> {
    fn meas(&mut self) -> bool {
        self.sys_pin.is_high() == self.trigger
    }
}

impl<P : InputPin> Interruptor for RawEndSwitch<P> {
    fn dir(&self) -> Option<Direction> {
        self._dir
    }

    fn check(&mut self, _gamma : &Arc<AtomicF32>) -> Option<InterruptReason> {
        if let Some(pin) = &mut self.sys_pin {
            // unwraping unsafe is safe, as no error can occur
            if unsafe { pin.is_high().unwrap_unchecked() } == self.trigger {    
                Some(InterruptReason::EndReached)
            } else {
                None
            }
        } else {
            Some(InterruptReason::Error) // TODO: add error
        }
    }
}

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

// #########################
// #    IMPLEMENTATIONS    #
// #########################
    // `UniPin` - Implementation
    pub struct EndSwitch {
        pin : u8,
        switch : RawEndSwitch<UniInPin>
    }

    impl EndSwitch {
        fn new(pin : u8) -> Self {
            Self {
                pin,
                switch: None
            }
        }
    }

    impl Setup for EndSwitch {
        fn setup(&mut self) -> Result<(), crate::Error> {
            self.switch = Some(
                RawEndSwitch::new(

                )
            );
            Ok(())
        }
    }
//