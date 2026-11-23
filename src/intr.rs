use alloc::boxed::Box;

use syunit::*;

/// A trait for structs that help interrupting or watching movement processes, the most common use are measurement systems
pub trait Interruptor {
    /// Direction of the interruptor
    /// - If `None` the interruptor is not dependent on a movement direction
    /// - If `Some` the interruptor is only active when moving in the given direction
    /// 
    /// ### Temporary dependence
    /// 
    /// If an interruptor was previously triggered by a movement, the control applies a temporary direction that lasts as long as
    /// the interruptor is triggered. Otherwise a not direction dependent switch would block movements completely
    fn dir(&self) -> Option<Direction>;
    
    /// Set temporary direction used to prevent locking of the axis
    /// 
    /// - A `Some` value sets the direction
    /// - A `None` value resets the direction
    /// 
    /// See `dir` for information about temporary direction
    fn set_temp_dir(&mut self, dir_opt : Option<Direction>);

    /// Runs a check of the movement process and Interrupts if it has a reason to
    fn check(&mut self) -> Option<InterruptReason>;
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

/// Represents an interruptible component, meaning `Interruptors` can be attached to modify the movement process
pub trait Interruptible {
    /// Add an interruptor to the component, often used for measurements or other processes checking the movement
    fn add_interruptor(&mut self, interruptor : Box<dyn Interruptor + Send>);

    /// Calls `add_interruptor` on an owned object
    fn add_interruptor_il(mut self, interruptor : Box<dyn Interruptor + Send>) -> Self 
    where 
        Self : Sized 
    {
        self.add_interruptor(interruptor);
        self
    }

    /// Returns the stored interrupt reason if there is any (returns `None` otherwise)
    /// 
    /// # Note
    /// 
    /// Executing this function will replace the reason with `None`, so if you need to access the value multiple times, you have to store it yourself
    fn fetch_intr_reason(&mut self) -> Option<InterruptReason>;
}