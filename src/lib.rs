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
    #[cfg(feature = "testing")]
    pub mod testing;
    #[cfg(feature = "testing")]
    pub use testing::*;
// 


// #######################
// #    ActuatorError    #
// #######################
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

        // Time errors
            /// The `Time` given is invalid somehow, depending on the context, see the function description
            InvalidTime(U::Time),
        // 

        // IO 
            /// Something is wrong with the IO connections of the actuator (PINs etc.)
            IOError,
            PinError,
            InterfaceError,
        // 

        // Load
            /// The component has been overloaded
            Overload,
            /// Too much force has been loaded onto the component `(0: Given force, 1: Maximum force)`
            ForceOverload(U::Force, U::Force)
        // 
    }

    impl<U : UnitSet> core::fmt::Display for ActuatorError<U> {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            f.write_fmt(format_args!("ActuatorError: {:?}", self))
        }
    }

    // TODO: Implement std errors
    // impl std::error::Error for ActuatorError { }
//