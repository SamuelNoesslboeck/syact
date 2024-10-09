use embedded_hal::digital::InputPin;
use serde::{Serialize, Deserialize};
use syunit::*;

use crate::act::{Interruptor, InterruptReason};
use crate::meas::Measurable;

/// A simple endswitch that can trigger when reaching a destination
#[derive(Serialize, Deserialize)]
pub struct EndStop<P : InputPin> {
    trigger : bool,
    _dir : Option<Direction>, 
    temp_dir : Option<Direction>,

    #[serde(skip)]
    sys_pin : P
}

impl<P : InputPin> EndStop<P> {
    /// Creates a new end switch
    pub fn new(trigger : bool, dir : Option<Direction>, sys_pin : P) -> Self {
        Self {
            trigger,
            _dir: dir,
            temp_dir: None,

            sys_pin
        }
    }
}

impl<P : InputPin> Measurable<bool> for EndStop<P> {
    type Error = P::Error; 

    fn measure(&mut self) -> Result<bool, Self::Error> {
        self.sys_pin.is_high().map(|v| v == self.trigger)
    }
}

impl<P : InputPin> Interruptor for EndStop<P> {
    fn dir(&self) -> Option<Direction> {
        self._dir.or(self.temp_dir)
    }

    fn set_temp_dir(&mut self, dir_opt : Option<Direction>) {
        self.temp_dir = dir_opt;
    }

    fn check(&mut self, _gamma : Gamma) -> Option<InterruptReason> {
        // unwraping unsafe is safe, as no error can occur
        if unsafe { self.sys_pin.is_high().unwrap_unchecked() } == self.trigger {    
            Some(InterruptReason::EndReached)
        } else {
            None
        }
    }
}