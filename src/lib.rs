#![doc = include_str!("../README.md")]
#![crate_name = "syact"]
#![no_std]
// #![deny(missing_docs)] 

// Modules
extern crate alloc;

// ####################
// #    SUBMODULES    #
// ####################
    /// Everything about actuators that work asynchronously
    mod asyn;
    pub use asyn::{AsyncActuator, PwmDcDriver};

    /// Common components and their implementations, like gears, linear axes etc.
    pub mod comps;

    /// Structs for storing characteristics of stepper motors and so on
    pub mod data;

    /// Interuption of components for measurements and other purposes
    pub mod intr; 
    pub use intr::{Interruptible, InterruptReason};

    /// Functions and Structs for taking measurements with a robot for e.g. position calculation
    pub mod rout;

    /// Component parent relations and their implementation
    pub mod parent;
    pub use parent::{ActuatorParent, RatioActuatorParent};

    /// Everything about actuators that work synchronously
    pub mod sync;
    pub use sync::{SyncActuator, AdvSyncActuator}; 

    /// Embed unit system library
    pub mod prelude;
    pub use syunit as units;

    // Testing
    #[cfg(feature = "__doctest_helper")]
    pub mod testing;
    #[cfg(feature = "__doctest_helper")]
    pub use testing::*;
// 


use syunit::*;

/// General Error type for `SyncActuators`
#[derive(Clone, Debug)]
pub enum ActuatorError<U : UnitSet> {
    /// The relative distance given is invalid, could be out of range or a number error
    InvaldRelativeDistance(U::Distance),
    /// The given position is out of bounds, either by mechanical properties or previously set limits. 
    /// Also gives a value on how much it is out of bounds
    PosOutOfBounds(U::Position, U::Distance),

    // Velocity errors
        /// The velocity given is invalid somehow, depending on the context (see the function description)
        InvalidVelocity(U::Velocity),
        /// The velocity given is too high, depending on the context, see the function description
        /// - 0: [U::Velocity] - The given velocity
        /// - 1: [U::Velocity] - The maximum velocity
        VelocityTooHigh(U::Velocity, U::Velocity),
    //

    // Acceleration
        /// The [Acceleration] given is invalid somehow, depending on the context (see the function description)
        InvalidAcceleration(U::Acceleration),
    // 

    // Jolt
        /// The `Jolt` given is invalid somehow, depending on the context (see the function description)
        InvalidJolt(U::Jolt),
    // 

    // Time 
        /// The `Time` given is invalid somehow, depending on the context, see the function description
        InvalidTime(U::Time),
    // 

    // IO 
        /// Something is wrong with the IO connections of the actuator
        IOError,
        /// Something is wrong with one of the hardware pins used by the actuator 
        PinError,
        /// Something is wrong with an interface used by the actuator
        InterfaceError,
    // 

    /// The component has been interrupted during the last movement operation and said interrrupt was not attended, meaning it might not be intentional
    UnattendedInterrupt(InterruptReason),
    /// The actuator has been overloaded
    Overload,
    /// Another error has happened that does not fit any of the categories above, see the underlying functions' description for more info
    Other
}

impl<U : UnitSet> core::fmt::Display for ActuatorError<U> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_fmt(format_args!("ActuatorError: {:?}", self))
    }
}

// TODO: Implement std errors
// impl std::error::Error for ActuatorError { }