use alloc::boxed::Box;
use alloc::sync::Arc;

use syunit::*;

// ####################
// #    SUBMODULES    #
// ####################
    /// A module for async components like a basic DC-motor. These components cannot move certain distances or to absolute positions
    pub mod asyn;
    pub use asyn::AsyncActuator;

    mod comps;
    pub use comps::{Conveyor, Gear, LinearAxis};

    /// A module for component groups, as they are used in various robots. The components are all sharing the same 
    /// [StepperConfig](crate::data::StepperConfig) and their movements are coordinated. 
    pub mod group;
    pub use group::{ActuatorGroup, SyncActuatorGroup};

    /// ALl the parent structures used
    pub mod parent;

    /// Everything concerning servo-motors
    pub mod servo;
    pub use servo::MiniServo;

    /// Stepper motors and their unique methods and traits
    pub mod stepper;
    pub use stepper::{StepperActuator, StepperMotor};
//

// Macros
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
            pub trait $name : $trait1 + $trait2 + $trait3 + $trait4{ }
            impl<T : $trait1 + $trait2 + $trait3 + $trait4> $name for T { }
        };
    }
// 

// #####################
// #    Interruptor    #
// #####################
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
        fn check(&mut self, abs_pos : AbsPos) -> Option<InterruptReason>;
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
    pub trait Interruptible {
        /// Add an interruptor to the component, often used for measurements or other processes checking the movement
        fn add_interruptor(&mut self, interruptor : Box<dyn Interruptor + Send>);

        /// Calls `add_interruptor` on an owned object
        fn add_interruptor_inline(mut self, interruptor : Box<dyn Interruptor + Send>) -> Self 
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
    pub enum ActuatorError {
        /// The rel_dist distance given is invalid
        InvaldRelativeDistance(RelDist),

        // Velocity errors
            /// The velocity given is invalid somehow, depending on the context (see the function description)
            InvalidVelocity(Velocity),
            /// The velocity given is too high, depending on the context, see the function description
            /// 0: [Velocity] - The given velocity
            /// 1: [Velocity] - The velocity
            VelocityTooHigh(Velocity, Velocity),
        //

        // Acceleration
            /// The [Acceleration] given is invalid somehow, depending on the context (see the function description)
            InvalidAcceleration(Acceleration),
        // 

        // Jolt
            /// The `Jolt` given is invalid somehow, depending on the context (see the function description)
            InvalidJolt(Jolt),
        // 

        // Time errors
            /// The `Time` given is invalid somehow, depending on the context, see the function description
            InvalidTime(Time),
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

    impl core::fmt::Display for ActuatorError {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            f.write_fmt(format_args!("{:?}", self))
        }
    }

    // TODO: Implement std errors
    // impl std::error::Error for ActuatorError { }
//

// ###########################
// #    Advanced Actuator    #
// ###########################
    /// An advanced actuator allows applying loads that alter the actuators movement
    pub trait AdvancedActuator {
        // Load calculation
            // TODO: Documentation sucks
            /// Will always be positive
            fn force_gen(&self) -> Force;

            // TODO: Documentation sucks
            /// Positive means CW direction
            fn force_dir(&self) -> Force;

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
            /// const FORCE : Force = Force(0.2);
            /// 
            /// // Create a new gear bearing (implements SyncActuator)
            /// let mut gear = Gear::new(
            ///     // Stepper Motor as subcomponent (also implements SyncActuator)
            ///     Stepper::default(), 
            /// 0.5);    // Ratio is set to 0.5, which means for each radian the motor moves, the bearing moves for half a radian
            /// 
            /// gear.apply_gen_force(FORCE);
            /// 
            /// assert_eq!(AbsPos(2.0), gear.abs_pos_for_child(AbsPos(1.0)));
            /// assert_eq!(Force(0.1), gear.child().force_gen());     // Forces get smaller for smaller gears
            /// ```
            fn apply_gen_force(&mut self, force : Force) -> Result<(), ActuatorError>;

            // TODO: Documentation sucks
            /// Value positive in CW direction
            fn apply_dir_force(&mut self, force : Force) -> Result<(), ActuatorError>;

            // Inertia
            /// Returns the inertia applied to the component
            fn inertia(&self) -> Inertia;
            
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
            /// const INERTIA : Inertia = Inertia(4.0);
            /// 
            /// // Create a new gear bearing (implements SyncActuator)
            /// let mut gear = Gear::new(
            ///     // Stepper Motor as subcomponent (also implements SyncActuator)
            ///     Stepper::default(), 
            /// 0.5);    // Ratio is set to 0.5, which means for each radian the motor moves, the bearing moves for half a radian
            /// 
            /// // Applies the inertia to the gearbearing component
            /// gear.apply_inertia(INERTIA);
            /// 
            /// assert_eq!(AbsPos(2.0), gear.abs_pos_for_child(AbsPos(1.0)));
            /// assert_eq!(Inertia(1.0), gear.child().inertia());
            /// ```
            fn apply_inertia(&mut self, inertia : Inertia) -> Result<(), ActuatorError> ;
        // 
    }
// 

// ######################
// #    SyncActuator    #
// ######################
    /// The state of a `SyncActuator` is used to control the component while it is moving and to get data about the current movement
    pub trait SyncActuatorState {
        /// Returns the current absolute position of the actuator
        fn abs_pos(&self) -> AbsPos; 

        /// Returns whether the actuator is currently moving or not
        fn moving(&self) -> bool;

        // Actions
            /// Halt the actuator
            fn halt(&self);

            /// Interrupt the movement of the actuator
            fn interrupt(&self);
        // 
    }   

    /// Trait for defining controls and components of synchronous actuators
    /// 
    /// # Parent components
    /// 
    /// Components can have multiple layers, for example take a stepper motor with a geaerbox attached to it. The stepper motor and both combined will be a component, the later having 
    /// the stepper motor component defined as it's parent component. (See [Gear])
    pub trait SyncActuator {
        // Position & Velocity
            /// Returns the **absolute** position of the component.
            /// 
            /// ```rust
            /// use syact::prelude::*;
            /// 
            /// // Position of components
            /// const POS : AbsPos = AbsPos(10.0);
            /// 
            /// // Create a new cylinder (implements SyncActuator)
            /// let mut cylinder = LinearAxis::new(
            ///     // Stepper Motor as subcomponent (also implements SyncActuator)
            ///     Stepper::default(), 
            /// 0.5);    // Ratio is set to 0.5, which means for each radian the motor moves, the cylinder moves for 0.5 mm
            /// 
            /// cylinder.overwrite_abs_pos(POS);
            /// 
            /// assert!((cylinder.abs_pos() - POS).abs() < RelDist(0.05));      // Check with small tolerance
            /// ```
            fn abs_pos(&self) -> AbsPos;

            /// Overwrite the current **absolute** position of the component without triggering actual movements. 
            /// 
            /// Be aware that only full steps can be written in distance, meaning that for position comparision a 
            /// small tolerance has to be considered, as the value written won't be the exact abs_pos value given.
            /// 
            /// ```rust
            /// use syact::prelude::*;
            /// 
            /// // Position of components
            /// const POS : AbsPos = AbsPos(10.0);
            /// 
            /// // Create a new cylinder (implements SyncActuator)
            /// let mut cylinder = LinearAxis::new(
            ///     // Stepper Motor as subcomponent (also implements SyncActuator)
            ///     Stepper::default(), 
            /// 0.5);    // Ratio is set to 0.5, which means for each radian the motor moves, the cylinder moves for 0.5 mm
            /// 
            /// cylinder.overwrite_abs_pos(POS);
            /// 
            /// assert!((cylinder.abs_pos() - POS).abs() < RelDist(0.05));      // Check with small tolerance
            /// ```
            fn overwrite_abs_pos(&mut self, abs_pos : AbsPos);
        //

        // Velocity max
            /// Maximum velocity allowed by the user if specified
            fn velocity_max(&self) -> Option<Velocity>;

            /// Set the maximum allowed [Velocity]
            /// 
            /// ## Option
            /// 
            /// Set to `None` if no limit is wished
            fn set_velocity_max(&mut self, velocity_opt : Option<Velocity>) -> Result<(), ActuatorError>;
        // 

        // Acceleration
            /// Maximum acceleration that will be allowed, if specified by the user with `set_max_acceleration`
            fn acceleration_max(&self) -> Option<Acceleration>;

            /// Set the maximum allowed [Acceleration]
            /// 
            /// ## Option
            /// 
            /// Set to `None` if no limit is wished
            fn set_acceleration_max(&mut self, acceleration_opt : Option<Acceleration>) -> Result<(), ActuatorError>;
        // 

        // Jolt
            /// The maximum jolt, if specified by the user
            fn jolt_max(&self) -> Option<Jolt>;

            /// Set the maximum allowed `Jolt` 
            /// 
            /// ## Option
            /// 
            /// Set to `None` if no limit is wished
            fn set_jolt_max(&mut self, jolt_opt : Option<Jolt>) -> Result<(), ActuatorError>;
        // 

        // Position limits
            /// The minimum position limit of the actuator, if set
            fn limit_min(&self) -> Option<AbsPos>;

            /// The maximum position limit of the actuator, if set
            fn limit_max(&self) -> Option<AbsPos>;

            /// Returns if any limit positions have been reached. The value returned can either be radians or millimeters, 
            /// depending on the type of component.
            /// 
            /// # Limits
            /// 
            /// If the return value
            /// - greater than 0, the maximum has been reached by the returned amount
            /// - is smaller than 0, the minimum has been reached by the returned amount
            /// - equal to 0, no limit has been reached
            /// - NaN, no limit has been set yet
            /// 
            /// # Example 
            /// 
            /// ```rust
            /// use syact::prelude::*;
            /// 
            /// // Limits
            /// const LIM_MAX : AbsPos = AbsPos(1.0);
            /// const LIM_MIN : AbsPos = AbsPos(-2.0);
            /// 
            /// const LIM_MIN_LOWER : AbsPos = AbsPos(-3.0);
            /// 
            /// // Create a new gear bearing (implements SyncActuator)
            /// let mut gear = Gear::new(
            ///     // Stepper Motor as subcomponent (also implements SyncActuator)
            ///     Stepper::default(), 
            /// 0.5);    // Ratio is set to 0.5, which means for each radian the motor moves, the bearing moves for half a radian
            /// 
            /// gear.set_pos_limits(Some(LIM_MIN), Some(LIM_MAX));
            /// 
            /// assert_eq!(gear.resolve_pos_limits_for_abs_pos(AbsPos(1.5)), RelDist(0.5));     // Over the maximum
            /// assert_eq!(gear.resolve_pos_limits_for_abs_pos(AbsPos(0.5)), RelDist::ZERO);    // In range
            /// assert_eq!(gear.resolve_pos_limits_for_abs_pos(AbsPos(-4.0)), RelDist(-2.0));   // Under the minimum
            /// 
            /// gear.set_pos_limits(Some(LIM_MIN_LOWER), None);                // Overwriting only `min` limit
            /// 
            /// assert_eq!(gear.resolve_pos_limits_for_abs_pos(AbsPos(1.5)), RelDist(0.5));     // Over the maximum
            /// assert_eq!(gear.resolve_pos_limits_for_abs_pos(AbsPos(0.5)), RelDist::ZERO);    // In range
            /// assert_eq!(gear.resolve_pos_limits_for_abs_pos(AbsPos(-4.0)), RelDist(-1.0));   // Under the minimum, but less
            /// 
            /// gear.overwrite_pos_limits(Some(LIM_MIN_LOWER), None);              // Overwriting only both limits with [overwrite_pos_limits()]
            /// 
            /// assert_eq!(gear.resolve_pos_limits_for_abs_pos(AbsPos(1.5)), RelDist::ZERO);    // In range, as the `max` limit has been deleted
            /// assert_eq!(gear.resolve_pos_limits_for_abs_pos(AbsPos(0.5)), RelDist::ZERO);    // In range
            /// assert_eq!(gear.resolve_pos_limits_for_abs_pos(AbsPos(-4.0)), RelDist(-1.0));   // Under the minimum, but less
            /// ```
            fn resolve_pos_limits_for_abs_pos(&self, abs_pos : AbsPos) -> RelDist;

            /// Sets an endpoint in the current direction by modifying the components limits. For example, when the component is moving
            /// in the positive direction and the endpoint is set, this function will overwrite the current maximum limit with the current
            /// abs_pos value. The component is then not allowed to move in the current direction anymore. 
            fn set_endpos(&mut self, overwrite_abs_pos : AbsPos);

            /// Set the limits for the minimum and maximum angles that the component can reach, note that the limit will 
            /// be converted and transfered to the parent component if defined. 
            /// 
            /// Unlike [SyncActuator::overwrite_pos_limits()], this function does not overwrite the current `min` or `max` limits if they
            /// are set to `None`. 
            /// 
            /// ```rust
            /// use syact::prelude::*;
            /// 
            /// // Limits
            /// const LIM_MAX : AbsPos = AbsPos(1.0);
            /// const LIM_MIN : AbsPos = AbsPos(-2.0);
            /// 
            /// const LIM_MIN_LOWER : AbsPos = AbsPos(-3.0);
            /// 
            /// // Create a new gear bearing (implements SyncActuator)
            /// let mut gear = Gear::new(
            ///     // Stepper Motor as subcomponent (also implements SyncActuator)
            ///     Stepper::default(), 
            /// 0.5);    // Ratio is set to 0.5, which means for each radian the motor moves, the bearing moves for half a radian
            /// 
            /// gear.set_pos_limits(Some(LIM_MIN), Some(LIM_MAX));
            /// 
            /// assert_eq!(gear.resolve_pos_limits_for_abs_pos(AbsPos(1.5)), RelDist(0.5));     // Over the maximum
            /// assert_eq!(gear.resolve_pos_limits_for_abs_pos(AbsPos(0.5)), RelDist::ZERO);    // In range
            /// assert_eq!(gear.resolve_pos_limits_for_abs_pos(AbsPos(-4.0)), RelDist(-2.0));   // Under the minimum
            /// 
            /// gear.set_pos_limits(Some(LIM_MIN_LOWER), None);                // Overwriting only `min` limit
            /// 
            /// assert_eq!(gear.resolve_pos_limits_for_abs_pos(AbsPos(1.5)), RelDist(0.5));     // Over the maximum
            /// assert_eq!(gear.resolve_pos_limits_for_abs_pos(AbsPos(0.5)), RelDist::ZERO);    // In range
            /// assert_eq!(gear.resolve_pos_limits_for_abs_pos(AbsPos(-4.0)), RelDist(-1.0));   // Under the minimum, but less
            /// 
            /// gear.overwrite_pos_limits(Some(LIM_MIN_LOWER), None);              // Overwriting only both limits with [overwrite_pos_limits()]
            /// 
            /// assert_eq!(gear.resolve_pos_limits_for_abs_pos(AbsPos(1.5)), RelDist::ZERO);    // In range, as the `max` limit has been deleted
            /// assert_eq!(gear.resolve_pos_limits_for_abs_pos(AbsPos(0.5)), RelDist::ZERO);    // In range
            /// assert_eq!(gear.resolve_pos_limits_for_abs_pos(AbsPos(-4.0)), RelDist(-1.0));   // Under the minimum, but less
            /// ```
            fn set_pos_limits(&mut self, min : Option<AbsPos>, max : Option<AbsPos>);

            /// Set the limits for the minimum and maximum angles that the component can reach, note that the limit will 
            /// be converted and transfered to the parent component if this component has one. 
            /// 
            /// The difference to [SyncActuator::set_pos_limits()] is that this function **overwrites** the current limits set.
            /// 
            /// ```rust
            /// use syact::prelude::*;
            /// 
            /// // Limits
            /// const LIM_MAX : AbsPos = AbsPos(1.0);
            /// const LIM_MIN : AbsPos = AbsPos(-2.0);
            /// 
            /// const LIM_MIN_LOWER : AbsPos = AbsPos(-3.0);
            /// 
            /// // Create a new gear bearing (implements SyncActuator)
            /// let mut gear = Gear::new(
            ///     // Stepper Motor as subcomponent (also implements SyncActuator)
            ///     Stepper::default(), 
            /// 0.5);    // Ratio is set to 0.5, which means for each radian the motor moves, the bearing moves for half a radian
            /// 
            /// gear.set_pos_limits(Some(LIM_MIN), Some(LIM_MAX));
            /// 
            /// assert_eq!(gear.resolve_pos_limits_for_abs_pos(AbsPos(1.5)), RelDist(0.5));     // Over the maximum
            /// assert_eq!(gear.resolve_pos_limits_for_abs_pos(AbsPos(0.5)), RelDist::ZERO);    // In range
            /// assert_eq!(gear.resolve_pos_limits_for_abs_pos(AbsPos(-4.0)), RelDist(-2.0));   // Under the minimum
            /// 
            /// gear.set_pos_limits(Some(LIM_MIN_LOWER), None);                // Overwriting only `min` limit
            /// 
            /// assert_eq!(gear.resolve_pos_limits_for_abs_pos(AbsPos(1.5)), RelDist(0.5));     // Over the maximum
            /// assert_eq!(gear.resolve_pos_limits_for_abs_pos(AbsPos(0.5)), RelDist::ZERO);    // In range
            /// assert_eq!(gear.resolve_pos_limits_for_abs_pos(AbsPos(-4.0)), RelDist(-1.0));   // Under the minimum, but less
            /// 
            /// gear.overwrite_pos_limits(Some(LIM_MIN_LOWER), None);              // Overwriting only both limits with [overwrite_pos_limits()]
            /// 
            /// assert_eq!(gear.resolve_pos_limits_for_abs_pos(AbsPos(1.5)), RelDist::ZERO);    // In range, as the `max` limit has been deleted
            /// assert_eq!(gear.resolve_pos_limits_for_abs_pos(AbsPos(0.5)), RelDist::ZERO);    // In range
            /// assert_eq!(gear.resolve_pos_limits_for_abs_pos(AbsPos(-4.0)), RelDist(-1.0));   // Under the minimum, but less
            /// ```
            fn overwrite_pos_limits(&mut self, min : Option<AbsPos>, max : Option<AbsPos>);
        // 
    }
//

// #########################################
// #    SyncActuator - Extention traits    #
// #########################################
    // Movement
        /// Further defines a `SyncActuator`, extending it with blocking movement functions
        pub trait SyncActuatorBlocking : SyncActuator {
            // State
                /// Returns a reference to the actuators `SyncActuatorState`
                fn state(&self) -> &dyn SyncActuatorState;

                /// Returns an `Arc` reference counter to the given S
                fn clone_state(&self) -> Arc<dyn SyncActuatorState>;
            // 

            /// Moves the component by the relative distance as fast as possible, blocks the script until the movement is finshed
            fn drive_rel_blocking(&mut self, rel_dist : RelDist, speed : Factor) -> Result<(), ActuatorError>;

            /// Moves the component to the absolute position as fast as possible, blocks the script until the movement is finshed
            #[inline]
            fn drive_abs_blocking(&mut self, abs_pos : AbsPos, speed : Factor) -> Result<(), ActuatorError> {
                let rel_dist = abs_pos - self.abs_pos();
                self.drive_rel_blocking(rel_dist, speed)
            }
        }

        /// Further defines a `SyncActuator`, extending it with non-blocking movement functions
        pub trait SyncActuatorNB : SyncActuator {
            /// Moves the component by the relative distance as fast as possible, blocks the script until the movement is finshed
            fn drive_rel_nb(&mut self, rel_dist : RelDist, speed : Factor) -> Result<(), ActuatorError>;

            /// Moves the component to the absolute position as fast as possible, blocks the script until the movement is finshed
            fn drive_abs_blocking(&mut self, abs_pos : AbsPos, speed : Factor) -> Result<(), ActuatorError> {
                let rel_dist = abs_pos - self.abs_pos();
                self.drive_rel_nb(rel_dist, speed)
            }
        }
    // 
// 