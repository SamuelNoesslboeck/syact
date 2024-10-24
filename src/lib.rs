#![doc = include_str!("../README.md")]
#![crate_name = "syact"]
#![cfg_attr(not(any(test, feature = "testing")), no_std)]

// Rules
#![deny(missing_docs)]
#![cfg_attr(feature = "testing", allow(unused))]

// Modules
extern crate alloc;

// Private imports
use alloc::boxed::Box;

use syunit::*;

// ####################
// #    SUBMODULES    #
// ####################
    // Core
        /// Everything about actuators that work asynchronously
        pub mod asyn;
        pub use asyn::AsyncActuator;

        mod comps;
        pub use comps::{Conveyor, Gear, LinearAxis};

        /// Structs for storing characteristics of stepper motors and so on
        pub mod data;
        pub use data::{MicroSteps, StepperConst, StepperConfig};

        /// Functions and Structs for taking measurements with a robot for e.g. position calculation
        pub mod meas;

        /// Component parent relations and their implementation
        pub mod parent;
        pub use parent::{ActuatorParent, RatioActuatorParent};

        /// Everything about actuators that work synchronously
        pub mod sync;
        pub use sync::{SyncActuator, SyncActuatorState, SyncActuatorBlocking, SyncActuatorNB}; 
    // 

    /// Easy import of the functionalities
    pub mod prelude;

    /// Module with all the tests required to assure the library funcitons as intended
    #[cfg(feature = "testing")]
    pub mod tests;

    pub use syunit as units;
// 

// Macros
    // TODO: Improve docs for this macro
    /// Helper macro for merging multiple Actuator traits into one, useful for group implementation
    #[macro_export]
    macro_rules! merge_actuator_traits {
        ($name:ident, $trait1:ident, $trait2:ident) => {
            pub trait $name : $trait1 + $trait2 { }
            impl<T : $trait1 + $trait2> $name for T { }
        };
        ($name:ident, $trait1:ident, $trait2:ident, $trait3:ident) => {
            pub trait $name : $trait1 + $trait2 + $trait3 { }
            impl<T : $trait1 + $trait2 + $trait3> $name for T { }
        };
        ($name:ident, $trait1:ident, $trait2:ident, $trait3:ident, $trait4:ident) => {
            pub trait $name : $trait1 + $trait2 + $trait3 + $trait4 { }
            impl<T : $trait1 + $trait2 + $trait3 + $trait4> $name for T { }
        };
    }
// 

// #####################
// #    Interruptor    #
// #####################
    /// A trait for structs that help interrupting or watching movement processes, the most common use are measurement systems
    pub trait Interruptor<U : UnitSet = Rotary> {
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
        fn check(&mut self, pos : U::Position) -> Option<InterruptReason>;
    }

    /// Reasons why an interrupt was triggered
    #[derive(PartialEq, Eq, PartialOrd, Ord, Debug, Clone, Copy)]
    pub enum InterruptReason {
        /// A virtual end or a switch has been reached
        EndReached,
        /// The component has been overloaded
        Overload,
        /// Another error has occured
        Error
    }

    /// Represents an interruptible component, meaning `Interruptors` can be attached to modify the movement process
    pub trait Interruptible<U : UnitSet = Rotary> {
        /// Add an interruptor to the component, often used for measurements or other processes checking the movement
        fn add_interruptor(&mut self, interruptor : Box<dyn Interruptor<U> + Send>);

        /// Calls `add_interruptor` on an owned object
        fn add_interruptor_inline(mut self, interruptor : Box<dyn Interruptor<U> + Send>) -> Self 
        where 
            Self : Sized 
        {
            self.add_interruptor(interruptor);
            self
        }

        /// Returns the interrupt reason if there is any (returns `None` otherwise)
        /// 
        /// # Note
        /// 
        /// Executing this function will replace the reason with `None`, so if you need to access the value multiple times, you have to store it yourself
        fn intr_reason(&mut self) -> Option<InterruptReason>;
    }
//

// #######################
// #    ActuatorError    #
// #######################
    /// General Error type for `SyncActuators`
    #[derive(Clone, Debug)]
    pub enum ActuatorError<U : UnitSet = Rotary> {
        /// The rel_dist distance given is invalid
        InvaldRelativeDistance(U::Distance),

        // U::Velocity errors
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
        // 

        // Load
            /// The component has been overloaded
            Overload
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

// ###########################
// #    Advanced Actuator    #
// ###########################
    /// An advanced actuator allows applying loads that alter the actuators movement
    pub trait AdvancedActuator<U : UnitSet = Rotary> {
        // Load calculation
            // TODO: Documentation sucks
            /// Will always be positive
            fn force_gen(&self) -> U::Force;

            // TODO: Documentation sucks
            /// Positive means CW direction
            fn force_dir(&self) -> U::Force;

            /// Apply a load force to the component, slowing down movements 
            /// 
            /// ### General force
            /// 
            /// Force value will always be made positive, as it will be subtracted in the calculation no matter how 
            /// 
            /// ```rust
            /// use syact::prelude::*;
            /// 
            /// // Force to act upon the component
            /// const FORCE : NewtonMeters = NewtonMeters(0.2);
            /// 
            /// // Create a new gear bearing (implements AdvancedActuator)
            /// let mut gear = Gear::new(
            ///     // Stepper Motor as subcomponent (also implements AdvancedActuator), `default()` function is only available for tests!
            ///     Stepper::default(), 
            ///     // Ratio is set to 0.5, which means for each radian the motor moves, the bearing moves for half a radian
            ///     0.5
            /// );  
            /// 
            /// gear.apply_gen_force(FORCE);
            /// 
            /// assert_eq!(PositionRad(2.0), gear.pos_for_child(PositionRad(1.0)));
            /// assert_eq!(NewtonMeters(0.1), gear.child().force_gen());     // Resistance gets smaller for this gear ratio
            /// ```
            fn apply_gen_force(&mut self, force : U::Force) -> Result<(), ActuatorError<U>>;

            // TODO: Documentation sucks
            /// Value positive in CW direction
            fn apply_dir_force(&mut self, force : U::Force) -> Result<(), ActuatorError<U>>;

            // Inertia
            /// Returns the inertia applied to the component, see [AdvancedActuator::apply_inertia]
            fn inertia(&self) -> U::Inertia;
            
            /// Apply a load inertia to the component, slowing down movements
            /// 
            /// # Panics
            /// 
            /// Panics if no parent component or override of the function has been provided.
            /// 
            /// ```rust
            /// use syact::prelude::*;
            /// 
            /// // Inertia to act upon the component
            /// const INERTIA : KgMeter2 = KgMeter2(4.0);
            /// 
            /// // Create a new gear bearing (implements AdvancedActuator)
            /// let mut gear = Gear::new(
            ///     // Stepper Motor as subcomponent (also implements AdvancedActuator), `default()` function is only available for tests!
            ///     Stepper::default(), 
            /// 0.5);    // Ratio is set to 0.5, which means for each radian the motor moves, the bearing moves for half a radian
            /// 
            /// // Applies the inertia to the gearbearing component
            /// gear.apply_inertia(INERTIA);
            /// 
            /// assert_eq!(PositionRad(2.0), gear.pos_for_child(PositionRad(1.0)));
            /// assert_eq!(KgMeter2(1.0), gear.child().inertia());          // Inertias get smaller with the ratio^2 !
            /// ```
            fn apply_inertia(&mut self, inertia : U::Inertia) -> Result<(), ActuatorError<U>> ;
        // 
    }

    /// An actuator that has a defined time to move for a PTP (Point-To-Point) movement
    pub trait DefinedActuator<U : UnitSet = Rotary> {
        /// The time required to perform a certain PTP (Point-To-Point movement)
        fn ptp_time_for_distance(&self, abs_pos_0 : U::Position, abs_pos_t : U::Position) -> U::Time;
    }
// 