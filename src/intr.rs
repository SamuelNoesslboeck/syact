use core::cell::Cell;

use alloc::boxed::Box;
use alloc::vec::Vec;

use syunit::*;

use crate::data::ActuatorVars;
use crate::ActuatorError;

/* Submodules */
    mod endstop;
    pub use endstop::EndStop;
/**/

/// Function pointer type for interruptor functions
#[allow(type_alias_bounds)]
pub type InterruptFunc<U : UnitSet> = dyn Fn(&ActuatorVars<U>) -> Result<Option<InterruptReason>, ActuatorError<U>> + Send;

/// An interruptor equiped with a custom [InterruptFunc] that can be used to embed hardware stops etc.
pub struct Interruptor<U : UnitSet> {
    func : Box<InterruptFunc<U>>,
    /// Optionally restrict the interruptor by direction
    dir : Option<Direction>,
    /// Temporary direction that ensures the interruptor gets an effective direction for a short time after it has been triggered
    /// to prevent stalls
    temp_dir : Cell<Option<Direction>>
}

impl<U : UnitSet> Interruptor<U> {
    /// Creates a new interruptor with the given `func`, optionally restricting it to an operation direction using `dir`
    pub fn new(func : Box<InterruptFunc<U>>, dir : Option<Direction>) -> Self {
        Self {
            func, 
            dir,
            temp_dir: Cell::new(None)
        }
    }

    /// The current effective direction, will be temporary set if not done so by the user to prevent stalling
    pub fn effective_dir(&self) -> Option<Direction> {
        self.dir.or(self.temp_dir.get())
    }   

    /// Checks if the interruptor has been triggered
    pub fn check(&self, vars : &ActuatorVars<U>, dir : Direction) -> Result<Option<InterruptReason>, ActuatorError<U>> {
        // Interruptor gets skipped if the actuator is currently not moving in it's effective direction
        if let Some(eff_dir) = self.effective_dir() {
            if eff_dir != dir {
                return Ok(None);   
            }
        }

        let reas = self.func.as_ref()(vars)?;
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

/// A convenient interrupt handler for `SyncActuators` to implement
pub struct InterruptHandler<U : UnitSet> {
    intr_list : Vec<Interruptor<U>>,
    last_intr_reason : Cell<Option<InterruptReason>>
}

impl<U : UnitSet> InterruptHandler<U> {
    /// Creates a new `InterruptHandler` with the given list of interruptors
    pub fn new(intr_list : Vec<Interruptor<U>>) -> Self {
        Self {
            intr_list,
            last_intr_reason: Cell::new(None)
        }
    }
    
    /// Creates a new `InterruptHandler` without any interruptors yet
    pub fn empty() -> Self {
        Self::new(Vec::new())
    }

    // TODO: Maybe handle this differently in the future?
    /// Adds a new interruptor to the handler
    pub fn add_intr(&mut self, intr : Interruptor<U>) {
        self.intr_list.push(intr);
    }

    /// Checks all interruptors in the order they have been added or their position in the array.
    /// As soon as a single one fires the function terminates and fires the interrupt. 
    /// 
    /// Throws an error if there is already an unattended interrupt stored from previous checks as it was very likely unintentional
    pub fn check_all(&self, vars : &ActuatorVars<U>, dir : Direction) -> Result<Option<InterruptReason>, ActuatorError<U>> {
        // If there is already an interrupt present, fire an error as the last one has not been attended and was unintentional probably
        if let Some(reason) = self.last_intr_reason.get() {
            return Err(ActuatorError::UnattendedInterrupt(reason))
        }

        for intr in &self.intr_list {
            let res = intr.check(vars, dir)?; 
            if res.is_some() {
                self.last_intr_reason.set(res);
                return Ok(res);
            }
        }

        Ok(None)
    }

    /// Fires the given interrupt manually, throwing an error if there is already an unattended one present
    pub fn fire_interrupt_manually(&self, new_reason : InterruptReason) -> Result<(), ActuatorError<U>> {
        // If there is already an interrupt present, fire an error as the last one has not been attended and was unintentional probably
        if let Some(reason) = self.last_intr_reason.get() {
            return Err(ActuatorError::UnattendedInterrupt(reason))
        }

        self.last_intr_reason.set(Some(new_reason));
        Ok(())
    }

    /// Returns the stored interrupt reason if there is any (returns `None` otherwise) and leaves `None` in its' place
    pub fn take_intr_reason(&self) -> Option<InterruptReason> {
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
    fn take_intr_reason(&self) -> Option<InterruptReason>;
}