use core::cell::RefCell;

use alloc::boxed::Box;
use embedded_hal::digital::InputPin;

use syunit::*;

use crate::{ActuatorError, InterruptReason, SyncActuator};
use crate::intr::Interruptor;

/// A simple endswitch that can trigger when reaching a destination
pub struct EndStop<P : InputPin + 'static> {
    pub trigger : bool,
    pub sys_pin : RefCell<P>,

    dir : Option<Direction>
}

impl<P : InputPin + 'static> EndStop<P> {
    /// Creates a new end switch with the following parameters
    /// - `trigger`: Whether the switch should trigger on a `HIGH` or `LOW`
    /// - `dir`: Optionally restrict the trigger by a moving direction, means it can only fire if the actuator
    /// is moving in the given direction
    /// - `sys_pin`: The pin which input should be read
    pub fn new(trigger : bool, dir : Option<Direction>, sys_pin : P) -> Self {
        Self {
            trigger,
            dir: dir,

            sys_pin: RefCell::new(sys_pin)
        }
    }

    pub fn create_intr<A : SyncActuator>(self) -> Interruptor<A> {
        Interruptor::new(Box::new(move |_| {
            if self.sys_pin.borrow_mut().is_high()
                .map_err(|_| ActuatorError::PinError)? == self.trigger 
            {    
                Ok(Some(InterruptReason::HardwareEndReached))
            } else {
                Ok(None)
            }
        }), self.dir)
    }
}