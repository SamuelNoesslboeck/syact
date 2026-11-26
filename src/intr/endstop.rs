use core::cell::RefCell;

use alloc::boxed::Box;
use embedded_hal::digital::InputPin;

use syunit::*;

use crate::{ActuatorError, InterruptReason, SyncActuator};
use crate::intr::Interruptor;

/// A simple endswitch that can trigger when reaching a destination
pub struct EndStop<P : InputPin + Send + 'static> {
    /// Whether the endstop triggers on `HIGH` or `LOW` (usually `LOW` so `false`)
    pub trigger : bool,
    /// The optionally set effective direction of the endstop
    pub dir : Option<Direction>,
    /// The hardware-pin connected to the `EndStop`
    pub sys_pin : RefCell<P>
}

impl<P : InputPin + Send + 'static> EndStop<P> {
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

    /// Creates an interruptor out of the endstop
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