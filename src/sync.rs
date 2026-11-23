use syunit::*;

use crate::ActuatorError;
use crate::data::ActuatorVars;

/// Everything concerning servo-motors
pub mod servo;
pub use servo::MiniServo;

/// Trait for standardizing actuators with synchronous movement
/// 
/// # Parent components
/// 
/// Components can have multiple layers, for example take a stepper motor with a geaerbox attached to it. The stepper motor and both combined will be a component, the later having 
/// the stepper motor component defined as it's parent component. (See [Gear])
pub trait SyncActuator<U : UnitSet = Rotary> {
    /* Accessors */
        /// Read-only accessor of the `ActuatorVars` of this actuator, defining this function automatically implements related functions
        /// like the application of loads if no additional overhead is required
        /// 
        /// It is not recommended to call functions, set loads etc. with this accessor, as the components are likely to have some overhead included
        fn acc_vars<F, R>(&self, f : F) -> R
        where
            F : FnOnce(&ActuatorVars<U>) -> R;

        /// Mutable accessor of the `ActuatorVars` of this actuator, defining this function automatically implements related functions
        /// like the application of loads if no additional overhead is required
        /// 
        /// It is not recommended to call functions, set loads etc. with this accessor, as the components are likely to have some overhead included
        fn acc_vars_mut<F, R>(&mut self, f : F) -> R
        where
            F : FnOnce(&mut ActuatorVars<U>) -> R;
    /**/

    /* Position */
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

    /* Status */
        /// Returns the current moving direction or the direction of last movement
        fn direction(&self) -> Direction;

        /// Returns the current velocity of the actuator
        fn velocity(&self) -> U::Velocity;

        /// Whether the actuator is currently moving
        fn is_moving(&self) -> bool;
    /**/

    /* Velocity Max */
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
        fn velocity_max(&self) -> Option<U::Velocity> {
            self.acc_vars(|vars| vars.velocity_max)
        }

        /// Set the maximum allowed velocity
        /// 
        /// ## Option
        /// 
        /// Set to `None` if no limit is wished
        fn set_velocity_max(&mut self, velocity_opt : Option<U::Velocity>) -> Result<(), ActuatorError<U>> {
            self.acc_vars_mut(
                |vars| vars.velocity_max = velocity_opt
            );
            Ok(())
        }
    /**/

    /* Acceleration Max */
        /// Maximum acceleration that will be allowed, if specified by the user with `set_max_acceleration`
        fn acceleration_max(&self) -> Option<U::Acceleration> {
            self.acc_vars(|vars| vars.acceleration_max)
        }

        /// Set the maximum allowed acceleration
        /// 
        /// ## Option
        /// 
        /// Set to `None` if no limit is wished
        fn set_acceleration_max(&mut self, acceleration_opt : Option<U::Acceleration>) -> Result<(), ActuatorError<U>> {
            self.acc_vars_mut(
                |vars| vars.acceleration_max = acceleration_opt
            );
            Ok(())
        }
    /**/

    /* Jolt Max */ 
        /// The maximum jolt, if specified by the user
        fn jolt_max(&self) -> Option<U::Jolt> {
            self.acc_vars(|vars| vars.jolt_max)
        }

        /// Set the maximum allowed `Jolt` 
        /// 
        /// ## Option
        /// 
        /// Set to `None` if no limit is wished
        fn set_jolt_max(&mut self, jolt_opt : Option<U::Jolt>) -> Result<(), ActuatorError<U>> {
            self.acc_vars_mut(
                |vars| vars.jolt_max = jolt_opt
            );
            Ok(())
        }
    /**/ 

    /* Position limits */
        /// The minimum position limit of the actuator if set
        fn limit_min(&self) -> Option<U::Position> {
            self.acc_vars(|vars| vars.pos_limits.min )
        }

        /// The maximum position limit of the actuator if set
        fn limit_max(&self) -> Option<U::Position> {
            self.acc_vars(|vars| vars.pos_limits.max )
        }

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
        fn set_endpos(&mut self, overwrite_abs_pos : U::Position) {
            self.overwrite_abs_pos(overwrite_abs_pos);

            let dir = self.direction().as_bool();
    
            self.set_pos_limits(
                if dir { None } else { Some(overwrite_abs_pos) },
                if dir { Some(overwrite_abs_pos) } else { None }
            )
        }

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
        ///     0.5   // Ratio is set to 0.5, which means for each radian the motor moves, the bearing moves for half a radian
        /// );   
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
        fn set_pos_limits(&mut self, min_opt : Option<U::Position>, max_opt : Option<U::Position>) {
            self.acc_vars_mut(|vars| {
                if min_opt.is_some() {
                    vars.pos_limits.min = min_opt;
                }
                if max_opt.is_some() {
                    vars.pos_limits.max = max_opt;
                }
            })
        }

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
        fn overwrite_pos_limits(&mut self, min_opt : Option<U::Position>, max_opt : Option<U::Position>) {
            self.acc_vars_mut(|vars| {
                vars.pos_limits.min = min_opt;
                vars.pos_limits.max = max_opt;
            })
        }
    // 

    /* Movement */
        /// Moves the component by the relative distance as fast as possible
        fn drive_rel(&mut self, rel_dist : U::Distance, speed : Factor) -> Result<(), ActuatorError<U>>;

        /// Moves the component to the absolute position as fast as possible
        fn drive_abs(&mut self, pos : U::Position, speed : Factor) -> Result<(), ActuatorError<U>> {
            let rel_dist = pos - self.pos();
            self.drive_rel(rel_dist, speed)
        }

        /// Moves the actuator with a factor of the maximum speed possible 
        fn drive_cont_factor(&mut self, speed : Factor, direction : Direction) -> Result<(), ActuatorError<U>>; 

        /// Start the movement process of the component with the given velocity `speed`, positive values for `speed` mean CW movement
        fn drive_cont_speed(&mut self, speed : U::Velocity) -> Result<(), ActuatorError<U>>;
    /**/
}


/// An advanced actuator allows applying loads that alter the actuators movement
pub trait AdvSyncActuator<U : UnitSet = Rotary> : SyncActuator<U> {
    // Load calculation
        /// Returns the general load force, meaning load force that applies in both directions
        /// (most of the time friction)
        fn load_force_gen(&self) -> U::Force {
            self.acc_vars(|vars| vars.load_force_gen)
        }

        /// Returns the directional load force, meaning load force that is directionally dependent
        fn load_force_dir(&self) -> U::Force {
            self.acc_vars(|vars| vars.load_force_dir)
        }

        /// Apply a general load force to the component, slowing down movements 
        /// 
        /// ### General force
        /// 
        /// Force value will always be made positive, as it will be subtracted in the calculation no matter in which direction the actuator moves, 
        /// perfectly fitted for loads like friction
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
        ///     DemoActuator::new(), 
        ///     // Ratio is set to 0.5, which means for each radian the motor moves, the bearing moves for half a radian
        ///     0.5
        /// );  
        /// 
        /// gear.apply_gen_force(FORCE);
        /// 
        /// assert_eq!(PositionRad(2.0), gear.pos_for_child(PositionRad(1.0)));
        /// assert_eq!(NewtonMeters(0.1), gear.child().load_force_gen());     // Resistance gets smaller for this gear ratio
        /// ```
        fn apply_gen_force(&mut self, force : U::Force) -> Result<(), ActuatorError<U>>;

        /// Apply a directional load force to the component, slowing down or assisting movements
        /// 
        /// ### Directional load force
        /// 
        /// A directional load force can be imagined like a weight attached to a rope and a pulley on the actuator. 
        /// Downwards the weight assists the motor in movement, upwards it slows the process down.
        /// 
        /// A positive value for this force means the force assists in `CW` direction and slows in `CCW` direction, 
        /// the other way around with negative forces
        fn apply_dir_force(&mut self, force : U::Force) -> Result<(), ActuatorError<U>>;

        /// Returns the inertia applied to the component, see [AdvancedActuator::apply_inertia]
        fn load_inertia(&self) -> U::Inertia {
            self.acc_vars(|vars| vars.load_inertia)
        }
        
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
        ///     DemoActuator::new(), 
        /// 0.5);    // Ratio is set to 0.5, which means for each radian the motor moves, the bearing moves for half a radian
        /// 
        /// // Applies the inertia to the gearbearing component
        /// gear.apply_load_inertia(INERTIA);
        /// 
        /// assert_eq!(PositionRad(2.0), gear.pos_for_child(PositionRad(1.0)));
        /// assert_eq!(KgMeter2(1.0), gear.child().load_inertia());          // Inertias get smaller with the ratio^2 !
        /// ```
        fn apply_load_inertia(&mut self, inertia : U::Inertia) -> Result<(), ActuatorError<U>> ;
    // 
}

/// An actuator that has a defined time to move for a PTP (Point-To-Point) movement
pub trait DefinedActuator<U : UnitSet = Rotary> {
    /// The time required to perform a certain PTP (Point-To-Point movement)
    fn ptp_time_for_distance(&self, abs_pos_0 : U::Position, abs_pos_t : U::Position) -> U::Time;
}
// 