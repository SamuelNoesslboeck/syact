use embedded_hal::digital::InputPin;

use syunit::*;

use crate::{Interruptor, InterruptReason};
use crate::meas::Measurable;

/// A simple endswitch that can trigger when reaching a destination
pub struct EndStop<P : InputPin> {
    pub trigger : bool,
    pub sys_pin : P,

    _dir : Option<Direction>, 
    temp_dir : Option<Direction>
}

impl<P : InputPin> EndStop<P> {
    /// Creates a new end switch with the following parameters
    /// - `trigger`: Whether the switch should trigger on a `HIGH` or `LOW`
    /// - `dir`: Optionally restrict the trigger by a moving direction, means it can only fire if the actuator
    /// is moving in the given direction
    /// - `sys_pin`: The pin which input should be read
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

    fn check(&mut self) -> Option<InterruptReason> {
        // TODO: Add errors to implementation!!!
        if unsafe { self.sys_pin.is_high().unwrap_unchecked() } == self.trigger {    
            Some(InterruptReason::LimitReached)
        } else {
            None
        }
    }
}