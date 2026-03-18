use core::pin::Pin;
use alloc::boxed::Box;
use alloc::sync::Arc;

use syunit::*;

use crate::ActuatorError;

// ####################
// #    SUBMODULES    #
// ####################
    /// Everything concerning servo-motors
    #[deprecated]
    pub mod servo;
    // pub use servo::MiniServo;
//

// ######################
// #    SyncActuator    #
// ######################
    /// The state of a `SyncActuator` is used to control the component while it is moving and to get data about the current movement
    pub trait SyncActuatorState<U : UnitSet = Rotary> {
        /// Returns the current absolute position of the actuator
        fn pos(&self) -> U::Position; 

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
    pub trait SyncActuator<U : UnitSet = Rotary> {
        // Position
            /// Returns the **absolute** position of the component in the components [Position unit](UnitSet::Position).
            /// 
            /// ```rust
            /// use syact::prelude::*;
            /// 
            /// // Position of components
            /// const POS : PositionMM = PositionMM(10.0);
            /// 
            /// // Create a new linear_axis (implements SyncActuator)
            /// let mut linear_axis = LinearAxis::new_belt_axis(
            ///     // Some demo actuator (implements SyncActuator)
            ///     DemoActuator::new(), 
            ///     Millimeters(0.5)    // The radius is set to 0.5, which means for each radian the motor moves, the linear_axis moves for 0.5 mm
            /// );    
            /// 
            /// linear_axis.overwrite_abs_pos(POS);
            /// 
            /// assert!((linear_axis.pos() - POS).abs() < Millimeters(0.001));      // Check with small tolerance requred for stepper motors
            /// ```
            fn pos(&self) -> U::Position;

            /// Overwrite the current **absolute** position of the component without triggering actual movements. 
            /// 
            /// ### Stepper tolerance
            /// 
            /// Be aware that only full steps can be written in distance, meaning that for position comparision a 
            /// small tolerance has to be considered, as the value written won't be the exact position value given.
            /// 
            /// ```rust
            /// use syact::prelude::*;
            /// 
            /// // Position of components
            /// const POS : PositionMM = PositionMM(10.0);
            /// 
            /// // Create a new linear_axis (implements SyncActuator)
            /// let mut linear_axis = LinearAxis::new_belt_axis(
            ///     // Some demo actuator (implements SyncActuator)
            ///     DemoActuator::new(), 
            ///     Millimeters(0.5)    // The radius is set to 0.5, which means for each radian the motor moves, the linear_axis moves for 0.5 mm
            /// );    
            /// 
            /// linear_axis.overwrite_abs_pos(POS);
            /// 
            /// assert!((linear_axis.pos() - POS).abs() < Millimeters(0.05));      // Check with small tolerance requred for stepper motors
            /// ```
            fn overwrite_abs_pos(&mut self, pos : U::Position);
        //

        // Velocity Max
            /// Maximum velocity allowed by the user if specified, otherwise will return `None`
            /// 
            /// ```rust
            /// use syact::prelude::*;
            /// 
            /// // Create a new demo actuator (only available when testing)
            /// let mut stepper = DemoActuator::new();   
            /// 
            /// assert_eq!(stepper.velocity_max(), None);
            /// ```
            fn velocity_max(&self) -> Option<U::Velocity>;

            /// Set the maximum allowed [U::Velocity]
            /// 
            /// ## Option
            /// 
            /// Set to `None` if no limit is wished
            fn set_velocity_max(&mut self, velocity_opt : Option<U::Velocity>) -> Result<(), ActuatorError<U>>;
        // 

        // Acceleration max
            /// Maximum acceleration that will be allowed, if specified by the user with `set_max_acceleration`
            fn acceleration_max(&self) -> Option<U::Acceleration>;

            /// Set the maximum allowed [Acceleration]
            /// 
            /// ## Option
            /// 
            /// Set to `None` if no limit is wished
            fn set_acceleration_max(&mut self, acceleration_opt : Option<U::Acceleration>) -> Result<(), ActuatorError<U>>;
        // 

        // Jolt max 
            /// The maximum jolt, if specified by the user
            fn jolt_max(&self) -> Option<U::Jolt>;

            /// Set the maximum allowed `Jolt` 
            /// 
            /// ## Option
            /// 
            /// Set to `None` if no limit is wished
            fn set_jolt_max(&mut self, jolt_opt : Option<U::Jolt>) -> Result<(), ActuatorError<U>>;
        // 

        // Position limits
            /// The minimum position limit of the actuator, if set
            fn limit_min(&self) -> Option<U::Position>;

            /// The maximum position limit of the actuator, if set
            fn limit_max(&self) -> Option<U::Position>;

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
            /// const LIM_MAX : PositionRad = PositionRad(1.0);
            /// const LIM_MIN : PositionRad = PositionRad(-2.0);
            /// 
            /// const LIM_MIN_LOWER : PositionRad = PositionRad(-3.0);
            /// 
            /// // Create a new gear bearing (implements SyncActuator)
            /// let mut gear = Gear::new(
            ///     // A demo actuator (implements SyncActuator)
            ///     DemoActuator::new(), 
            /// 0.5);    // Ratio is set to 0.5, which means for each radian the motor moves, the bearing moves for half a radian
            /// 
            /// gear.set_pos_limits(Some(LIM_MIN), Some(LIM_MAX));
            /// 
            /// assert_eq!(gear.resolve_pos_limits_for_abs_pos(PositionRad(1.5)), Radians(0.5));     // Over the maximum
            /// assert_eq!(gear.resolve_pos_limits_for_abs_pos(PositionRad(0.5)), Radians::ZERO);    // In range
            /// assert_eq!(gear.resolve_pos_limits_for_abs_pos(PositionRad(-4.0)), Radians(-2.0));   // Under the minimum
            /// 
            /// gear.set_pos_limits(Some(LIM_MIN_LOWER), None);                // Overwriting only `min` limit
            /// 
            /// assert_eq!(gear.resolve_pos_limits_for_abs_pos(PositionRad(1.5)), Radians(0.5));     // Over the maximum
            /// assert_eq!(gear.resolve_pos_limits_for_abs_pos(PositionRad(0.5)), Radians::ZERO);    // In range
            /// assert_eq!(gear.resolve_pos_limits_for_abs_pos(PositionRad(-4.0)), Radians(-1.0));   // Under the minimum, but less
            /// 
            /// gear.overwrite_pos_limits(Some(LIM_MIN_LOWER), None);              // Overwriting only both limits with [overwrite_pos_limits()]
            /// 
            /// assert_eq!(gear.resolve_pos_limits_for_abs_pos(PositionRad(1.5)), Radians::ZERO);    // In range, as the `max` limit has been deleted
            /// assert_eq!(gear.resolve_pos_limits_for_abs_pos(PositionRad(0.5)), Radians::ZERO);    // In range
            /// assert_eq!(gear.resolve_pos_limits_for_abs_pos(PositionRad(-4.0)), Radians(-1.0));   // Under the minimum, but less
            /// ```
            fn resolve_pos_limits_for_abs_pos(&self, pos : U::Position) -> U::Distance {
                                if let Some(ang) = self.limit_min() {
                    if pos < ang {
                        pos - ang
                    } else {
                        if let Some(ang) = self.limit_max() {
                            if pos > ang {
                                pos - ang
                            } else { 
                                U::Distance::ZERO
                            }
                        } else {
                            U::Distance::ZERO
                        }
                    }
                } else {
                    if let Some(ang) = self.limit_max() {
                        if pos > ang {
                            pos - ang
                        } else { 
                            U::Distance::ZERO
                        }
                    } else {
                        U::Distance::NAN
                    }
                }
            }

            /// Sets an endpoint in the current direction by modifying the components limits. For example, when the component is moving
            /// in the positive direction and the endpoint is set, this function will overwrite the current maximum limit with the current
            /// pos value. The component is then not allowed to move in the current direction anymore. 
            fn set_endpos(&mut self, overwrite_abs_pos : U::Position);

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
            /// const LIM_MAX : PositionRad = PositionRad(1.0);
            /// const LIM_MIN : PositionRad = PositionRad(-2.0);
            /// 
            /// const LIM_MIN_LOWER : PositionRad = PositionRad(-3.0);
            /// 
            /// // Create a new gear bearing (implements SyncActuator)
            /// let mut gear = Gear::new(
            ///     // A demo actuator (implements SyncActuator)
            ///     DemoActuator::new(), 
            /// 0.5);    // Ratio is set to 0.5, which means for each radian the motor moves, the bearing moves for half a radian
            /// 
            /// gear.set_pos_limits(Some(LIM_MIN), Some(LIM_MAX));
            /// 
            /// assert_eq!(gear.resolve_pos_limits_for_abs_pos(PositionRad(1.5)), Radians(0.5));     // Over the maximum
            /// assert_eq!(gear.resolve_pos_limits_for_abs_pos(PositionRad(0.5)), Radians::ZERO);    // In range
            /// assert_eq!(gear.resolve_pos_limits_for_abs_pos(PositionRad(-4.0)), Radians(-2.0));   // Under the minimum
            /// 
            /// gear.set_pos_limits(Some(LIM_MIN_LOWER), None);                // Overwriting only `min` limit
            /// 
            /// assert_eq!(gear.resolve_pos_limits_for_abs_pos(PositionRad(1.5)), Radians(0.5));     // Over the maximum
            /// assert_eq!(gear.resolve_pos_limits_for_abs_pos(PositionRad(0.5)), Radians::ZERO);    // In range
            /// assert_eq!(gear.resolve_pos_limits_for_abs_pos(PositionRad(-4.0)), Radians(-1.0));   // Under the minimum, but less
            /// 
            /// gear.overwrite_pos_limits(Some(LIM_MIN_LOWER), None);              // Overwriting only both limits with [overwrite_pos_limits()]
            /// 
            /// assert_eq!(gear.resolve_pos_limits_for_abs_pos(PositionRad(1.5)), Radians::ZERO);    // In range, as the `max` limit has been deleted
            /// assert_eq!(gear.resolve_pos_limits_for_abs_pos(PositionRad(0.5)), Radians::ZERO);    // In range
            /// assert_eq!(gear.resolve_pos_limits_for_abs_pos(PositionRad(-4.0)), Radians(-1.0));   // Under the minimum, but less
            /// ```
            fn set_pos_limits(&mut self, min : Option<U::Position>, max : Option<U::Position>);

            /// Set the limits for the minimum and maximum angles that the component can reach, note that the limit will 
            /// be converted and transfered to the parent component if this component has one. 
            /// 
            /// The difference to [SyncActuator::set_pos_limits()] is that this function **overwrites** the current limits set.
            /// 
            /// ```rust
            /// use syact::prelude::*;
            /// 
            /// // Limits
            /// const LIM_MAX : PositionRad = PositionRad(1.0);
            /// const LIM_MIN : PositionRad = PositionRad(-2.0);
            /// 
            /// const LIM_MIN_LOWER : PositionRad = PositionRad(-3.0);
            /// 
            /// // Create a new gear bearing (implements SyncActuator)
            /// let mut gear = Gear::new(
            ///     // A demo actuator (implements SyncActuator)
            ///     DemoActuator::new(), 
            /// 0.5);    // Ratio is set to 0.5, which means for each radian the motor moves, the bearing moves for half a radian
            /// 
            /// gear.set_pos_limits(Some(LIM_MIN), Some(LIM_MAX));
            /// 
            /// assert_eq!(gear.resolve_pos_limits_for_abs_pos(PositionRad(1.5)), Radians(0.5));     // Over the maximum
            /// assert_eq!(gear.resolve_pos_limits_for_abs_pos(PositionRad(0.5)), Radians::ZERO);    // In range
            /// assert_eq!(gear.resolve_pos_limits_for_abs_pos(PositionRad(-4.0)), Radians(-2.0));   // Under the minimum
            /// 
            /// gear.set_pos_limits(Some(LIM_MIN_LOWER), None);                // Overwriting only `min` limit
            /// 
            /// assert_eq!(gear.resolve_pos_limits_for_abs_pos(PositionRad(1.5)), Radians(0.5));     // Over the maximum
            /// assert_eq!(gear.resolve_pos_limits_for_abs_pos(PositionRad(0.5)), Radians::ZERO);    // In range
            /// assert_eq!(gear.resolve_pos_limits_for_abs_pos(PositionRad(-4.0)), Radians(-1.0));   // Under the minimum, but less
            /// 
            /// gear.overwrite_pos_limits(Some(LIM_MIN_LOWER), None);              // Overwriting only both limits with [overwrite_pos_limits()]
            /// 
            /// assert_eq!(gear.resolve_pos_limits_for_abs_pos(PositionRad(1.5)), Radians::ZERO);    // In range, as the `max` limit has been deleted
            /// assert_eq!(gear.resolve_pos_limits_for_abs_pos(PositionRad(0.5)), Radians::ZERO);    // In range
            /// assert_eq!(gear.resolve_pos_limits_for_abs_pos(PositionRad(-4.0)), Radians(-1.0));   // Under the minimum, but less
            /// ```
            fn overwrite_pos_limits(&mut self, min : Option<U::Position>, max : Option<U::Position>);
        // 

        // State
            /// Returns an `Arc` reference counter to the given S
            fn clone_state(&self) -> Arc<dyn SyncActuatorState<U>>;
        // 

        // Movement
            /// Moves the component by the relative distance as fast as possible
            async fn drive_rel(&mut self, rel_dist : U::Distance, speed : Factor) -> Result<(), ActuatorError<U>>;

            /// Moves the component to the absolute position as fast as possible
            async fn drive_abs(&mut self, pos : U::Position, speed : Factor) -> Result<(), ActuatorError<U>> {
                let rel_dist = pos - self.pos();
                self.drive_rel(rel_dist, speed).await
            }

            /// Moves the actuator with a factor of the maximum speed possible 
            async fn drive_factor(&mut self, speed : Factor, direction : Direction) -> Result<(), ActuatorError<U>>; 

            /// Start the movement process of the component with the given velocity `speed`, positive values for `speed` mean CW movement
            async fn drive_speed(&mut self, speed : U::Velocity) -> Result<(), ActuatorError<U>>;
        // 
    }
//