// Private imports
use alloc::sync::Arc;

use syunit::*;

use crate::ActuatorError;

// ####################
// #    SUBMODULES    #
// ####################
    /// Everything concerning servo-motors
    pub mod servo;
    pub use servo::MiniServo;

    /// Stepper motors and their unique methods and traits
    pub mod stepper;
    pub use stepper::{StepperActuator, StepperMotor};
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

            /// Starts the movement process of the component in the given direction with a given `speed` factor
            fn drive_factor(&mut self, speed : Factor, direction : Direction) -> Result<(), ActuatorError>; 

            /// Start the movement process of the component with the given velocity `speed`, positive values for `speed` mean CW movement
            fn drive_speed(&mut self, speed : Velocity) -> Result<(), ActuatorError>;
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