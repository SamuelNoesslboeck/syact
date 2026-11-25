use core::cell::Cell;

use alloc::boxed::Box;
use alloc::vec::Vec;

use syunit::*;

use crate::{ActuatorError, SyncActuator};

/* Submodules */
    mod endstop;
    pub use endstop::EndStop;
/**/

#[allow(type_alias_bounds)]
pub type InterruptFunc<A : SyncActuator> = dyn Fn(&A) -> Result<Option<InterruptReason>, ActuatorError<A::Units>>;

pub struct Interruptor<A : SyncActuator> {
    func : Box<InterruptFunc<A>>,
    /// Optionally restrict the interruptor by direction
    dir : Option<Direction>,
    /// Temporary direction that ensures the interruptor gets an effective direction for a short time after it has been triggered
    /// to prevent stalls
    temp_dir : Cell<Option<Direction>>
}

impl<A : SyncActuator> Interruptor<A> {
    pub fn new(func : Box<InterruptFunc<A>>, dir : Option<Direction>) -> Self {
        Self {
            func, 
            dir,
            temp_dir: Cell::new(None)
        }
    }

    pub fn effective_dir(&self) -> Option<Direction> {
        self.dir.or(self.temp_dir.get())
    }

    pub fn check(&self, act : &A, dir : Direction) -> Result<Option<InterruptReason>, ActuatorError<A::Units>> {
        // Interruptor gets skipped if the actuator is currently not moving in it's effective direction
        if let Some(eff_dir) = self.effective_dir() {
            if eff_dir != dir {
                return Ok(None);   
            }
        }

        let reas = self.func.as_ref()(act)?;
        if reas.is_some() {
            // Setting the temporary direction prevents a stall when using non-directional interruptors
            self.temp_dir.set(Some(dir));
            return Ok(reas);
        } else {
            self.temp_dir.set(None);
        }

        Ok(None)
    }
}

/// Reasons why an interrupt was triggered
#[derive(PartialEq, Eq, PartialOrd, Ord, Debug, Clone, Copy)]
pub enum InterruptReason {
    /// A hardware end has been reached (switch, etc.)
    HardwareEndReached,
    /// A virtual component limit has been reached
    LimitReached,
    /// The component has been overloaded
    Overload,
    /// Another error has occured
    Error
}

pub struct InterruptHandler<A : SyncActuator> {
    intr_list : Vec<Interruptor<A>>,
    last_intr_reason : Cell<Option<InterruptReason>>
}

impl<A : SyncActuator> InterruptHandler<A> {
    pub fn new() -> Self {
        Self {
            intr_list: Vec::new(),
            last_intr_reason: Cell::new(None)
        }
    }

    // TODO: Maybe handle this differently in the future?
    pub fn add_intr(&mut self, intr : Interruptor<A>) {
        self.intr_list.push(intr);
    }

    pub fn check_all(&self, act : &A, dir : Direction) -> Result<Option<InterruptReason>, ActuatorError<A::Units>> {
        for intr in &self.intr_list {
            let res = intr.check(act, dir)?; 
            if res.is_some() {
                self.last_intr_reason.set(res);
                return Ok(res);
            }
        }

        Ok(None)
    }

    /// Returns the stored interrupt reason if there is any (returns `None` otherwise)
    /// 
    /// # Note
    /// 
    /// Executing this function will replace the reason with `None`, so if you need to access the value multiple times, you have to store it yourself
    pub fn fetch_intr_reason(&self) -> Option<InterruptReason> {
        self.last_intr_reason.take()
    }
}

/// Represents an interruptible component, meaning `Interruptors` can be attached to modify the movement process
pub trait Interruptible {
    /// Returns the stored interrupt reason if there is any (returns `None` otherwise)
    /// 
    /// # Note
    /// 
    /// Executing this function will replace the reason with `None`, so if you need to access the value multiple times, you have to store it yourself
    fn fetch_intr_reason(&self) -> Option<InterruptReason>;
}