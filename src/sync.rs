use syunit::*;

use crate::ActuatorError;
use crate::data::{ActuatorState, ActuatorVars};

/// Everything concerning servo-motors
pub mod servo;
pub use servo::MiniServo;

/// Trait for standardizing actuators with synchronous movement
/// 
/// # Parent components
/// 
/// Components can have multiple layers, for example take a stepper motor with a geaerbox attached to it. The stepper motor and both combined will be a component, the later having 
/// the stepper motor component defined as it's parent component. (See [Gear])
pub trait SyncActuator {
    /// The [UnitSet] used by this actuator
    type Units : UnitSet;

    /* Accessors */
        /// Read-only accessor of the `ActuatorVars` of this actuator, defining this function automatically implements related functions
        /// like the application of loads if no additional overhead is required
        /// 
        /// It is not recommended to call functions, set loads etc. with this accessor, as the components are likely to have some overhead included
        #[doc(hidden)]
        fn __acc_vars<F, R>(&self, f : F) -> R
        where
            F : FnOnce(&ActuatorVars<Self::Units>) -> R;

        /// Mutable accessor of the `ActuatorVars` of this actuator, defining this function automatically implements related functions
        /// like the application of loads if no additional overhead is required
        /// 
        /// It is not recommended to call functions, set loads etc. with this accessor, as the components are likely to have some overhead included
        #[doc(hidden)]
        fn __acc_vars_mut<F, R>(&mut self, f : F) -> R
        where
            F : FnOnce(&mut ActuatorVars<Self::Units>) -> R;
    /**/

    /* State */
        /// Returns a copy of the current `ActuatorState`, useful for fetching if the actuator is moving and in which direction or it has 
        /// ran into an error when using concurrency
        fn state(&self) -> ActuatorState<Self::Units> {
            // Cloning is ok as state is only a small datatype
            self.__acc_vars(|vars| vars.state.clone())
        }

        /* Position */
            /// Returns the **absolute** position of the component in the components [Position unit](UnitSet::Position)
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
            /// linear_axis.overwrite_position(POS);
            /// 
            /// assert!((linear_axis.position() - POS).abs() < Millimeters(0.001));      // Check with small tolerance requred for stepper motors
            /// ```
            fn position(&self) -> <Self::Units as UnitSet>::Position {
                self.__acc_vars(|vars| vars.position)
            }

            /// Overwrite the current **absolute** position of the component without triggering actual movements
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
            /// linear_axis.overwrite_position(POS);
            /// 
            /// assert!((linear_axis.position() - POS).abs() < Millimeters(0.05));      // Check with small tolerance requred for stepper motors
            /// ```
            fn overwrite_position(&mut self, pos : <Self::Units as UnitSet>::Position) {
                self.__acc_vars_mut(|vars| vars.position = pos)
            }
        /**/
        

        /// Returns the current velocity of the actuator if known, otherwise `None`
        fn velocity(&self) -> Option<<Self::Units as UnitSet>::Velocity> {
            self.__acc_vars(|vars| vars.velocity)
        }

        /// Returns the current acceleration of the actuator if known, otherwise `None`
        fn acceleration(&self) -> Option<<Self::Units as UnitSet>::Acceleration> {
            self.__acc_vars(|vars| vars.acceleration)
        }
    /**/

    /* Velocity Max & Possible */
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
        fn velocity_max(&self) -> Option<<Self::Units as UnitSet>::Velocity> {
            self.__acc_vars(|vars| vars.velocity_max)
        }

        /// Set the maximum allowed velocity
        /// 
        /// ## Option
        /// 
        /// Set to `None` if no limit is wished
        fn set_velocity_max(&mut self, velocity_opt : Option<<Self::Units as UnitSet>::Velocity>) -> Result<(), ActuatorError<Self::Units>> {
            self.__acc_vars_mut(
                |vars| vars.velocity_max = velocity_opt
            );
            Ok(())
        }

        /// Core function that is implemented by the user, 
        /// should return the maximum possible velocity that can be provided by the actuator right now
        #[doc(hidden)]
        fn __vel_possible(&self) -> <Self::Units as UnitSet>::Velocity;

        /// Maximum possible acceleration considering actuator and user-set limitations
        fn vel_possible(&self) -> <Self::Units as UnitSet>::Velocity {
            self.velocity_max().unwrap_or(<Self::Units as UnitSet>::Velocity::INFINITY)
                .min(self.__vel_possible())
        }
    /**/

    /* Acceleration Max */
        /// Maximum acceleration that will be allowed, if specified by the user with `set_max_acceleration`
        fn acceleration_max(&self) -> Option<<Self::Units as UnitSet>::Acceleration> {
            self.__acc_vars(|vars| vars.acceleration_max)
        }

        /// Set the maximum allowed acceleration
        /// 
        /// ## Option
        /// 
        /// Set to `None` if no limit is wished
        fn set_acceleration_max(&mut self, acceleration_opt : Option<<Self::Units as UnitSet>::Acceleration>) -> Result<(), ActuatorError<Self::Units>> {
            self.__acc_vars_mut(
                |vars| vars.acceleration_max = acceleration_opt
            );
            Ok(())
        }
    /**/

    /* Jolt Max */ 
        /// The maximum jolt, if specified by the user
        fn jolt_max(&self) -> Option<<Self::Units as UnitSet>::Jolt> {
            self.__acc_vars(|vars| vars.jolt_max)
        }

        /// Set the maximum allowed `Jolt` 
        /// 
        /// ## Option
        /// 
        /// Set to `None` if no limit is wished
        fn set_jolt_max(&mut self, jolt_opt : Option<<Self::Units as UnitSet>::Jolt>) -> Result<(), ActuatorError<Self::Units>> {
            self.__acc_vars_mut(
                |vars| vars.jolt_max = jolt_opt
            );
            Ok(())
        }
    /**/ 

    /* Position limits */
        /// The minimum position limit of the actuator if set
        fn limit_min(&self) -> Option<<Self::Units as UnitSet>::Position> {
            self.__acc_vars(|vars| vars.pos_limits.min )
        }

        /// The maximum position limit of the actuator if set
        fn limit_max(&self) -> Option<<Self::Units as UnitSet>::Position> {
            self.__acc_vars(|vars| vars.pos_limits.max )
        }

        /// Returns if any limit positions have been reached, returning the about of distance it has been overstepped
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
        /// assert_eq!(gear.resolve_limits_for_pos(PositionRad(1.5)), Radians(0.5));     // Over the maximum
        /// assert_eq!(gear.resolve_limits_for_pos(PositionRad(0.5)), Radians::ZERO);    // In range
        /// assert_eq!(gear.resolve_limits_for_pos(PositionRad(-4.0)), Radians(-2.0));   // Under the minimum
        /// 
        /// gear.set_pos_limits(Some(LIM_MIN_LOWER), None);                // Overwriting only `min` limit
        /// 
        /// assert_eq!(gear.resolve_limits_for_pos(PositionRad(1.5)), Radians(0.5));     // Over the maximum
        /// assert_eq!(gear.resolve_limits_for_pos(PositionRad(0.5)), Radians::ZERO);    // In range
        /// assert_eq!(gear.resolve_limits_for_pos(PositionRad(-4.0)), Radians(-1.0));   // Under the minimum, but less
        /// 
        /// gear.overwrite_pos_limits(Some(LIM_MIN_LOWER), None);              // Overwriting only both limits with [overwrite_pos_limits()]
        /// 
        /// assert_eq!(gear.resolve_limits_for_pos(PositionRad(1.5)), Radians::ZERO);    // In range, as the `max` limit has been deleted
        /// assert_eq!(gear.resolve_limits_for_pos(PositionRad(0.5)), Radians::ZERO);    // In range
        /// assert_eq!(gear.resolve_limits_for_pos(PositionRad(-4.0)), Radians(-1.0));   // Under the minimum, but less
        /// ```
        fn resolve_limits_for_pos(&self, pos : <Self::Units as UnitSet>::Position) -> <Self::Units as UnitSet>::Distance {
            if let Some(ang) = self.limit_min() {
                if pos < ang {
                    return pos - ang;
                } 
            } 
                
            if let Some(ang) = self.limit_max() {
                if pos > ang {
                    return pos - ang;
                }
            }

            <Self::Units as UnitSet>::Distance::ZERO
        }

        /// Sets an endpoint in the current direction by modifying the components limits. For example, when the component is moving
        /// in the positive direction and the endpoint is set, this function will overwrite the current maximum limit with the current
        /// pos value. The component is then not allowed to move in the current direction anymore. 
        fn set_endpos(&mut self, new_abs_pos : <Self::Units as UnitSet>::Position, towards : Direction) {
            self.overwrite_position(new_abs_pos);
    
            self.set_pos_limits(
                if towards == Direction::CW { None } else { Some(new_abs_pos) },
                if towards == Direction::CW { Some(new_abs_pos) } else { None }
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
        /// assert_eq!(gear.resolve_limits_for_pos(PositionRad(1.5)), Radians(0.5));     // Over the maximum
        /// assert_eq!(gear.resolve_limits_for_pos(PositionRad(0.5)), Radians::ZERO);    // In range
        /// assert_eq!(gear.resolve_limits_for_pos(PositionRad(-4.0)), Radians(-2.0));   // Under the minimum
        /// 
        /// gear.set_pos_limits(Some(LIM_MIN_LOWER), None);                // Overwriting only `min` limit
        /// 
        /// assert_eq!(gear.resolve_limits_for_pos(PositionRad(1.5)), Radians(0.5));     // Over the maximum
        /// assert_eq!(gear.resolve_limits_for_pos(PositionRad(0.5)), Radians::ZERO);    // In range
        /// assert_eq!(gear.resolve_limits_for_pos(PositionRad(-4.0)), Radians(-1.0));   // Under the minimum, but less
        /// 
        /// gear.overwrite_pos_limits(Some(LIM_MIN_LOWER), None);              // Overwriting only both limits with [overwrite_pos_limits()]
        /// 
        /// assert_eq!(gear.resolve_limits_for_pos(PositionRad(1.5)), Radians::ZERO);    // In range, as the `max` limit has been deleted
        /// assert_eq!(gear.resolve_limits_for_pos(PositionRad(0.5)), Radians::ZERO);    // In range
        /// assert_eq!(gear.resolve_limits_for_pos(PositionRad(-4.0)), Radians(-1.0));   // Under the minimum, but less
        /// ```
        fn set_pos_limits(&mut self, min_opt : Option<<Self::Units as UnitSet>::Position>, max_opt : Option<<Self::Units as UnitSet>::Position>) {
            self.__acc_vars_mut(|vars| {
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
        /// assert_eq!(gear.resolve_limits_for_pos(PositionRad(1.5)), Radians(0.5));     // Over the maximum
        /// assert_eq!(gear.resolve_limits_for_pos(PositionRad(0.5)), Radians::ZERO);    // In range
        /// assert_eq!(gear.resolve_limits_for_pos(PositionRad(-4.0)), Radians(-2.0));   // Under the minimum
        /// 
        /// gear.set_pos_limits(Some(LIM_MIN_LOWER), None);                // Overwriting only `min` limit
        /// 
        /// assert_eq!(gear.resolve_limits_for_pos(PositionRad(1.5)), Radians(0.5));     // Over the maximum
        /// assert_eq!(gear.resolve_limits_for_pos(PositionRad(0.5)), Radians::ZERO);    // In range
        /// assert_eq!(gear.resolve_limits_for_pos(PositionRad(-4.0)), Radians(-1.0));   // Under the minimum, but less
        /// 
        /// gear.overwrite_pos_limits(Some(LIM_MIN_LOWER), None);              // Overwriting only both limits with [overwrite_pos_limits()]
        /// 
        /// assert_eq!(gear.resolve_limits_for_pos(PositionRad(1.5)), Radians::ZERO);    // In range, as the `max` limit has been deleted
        /// assert_eq!(gear.resolve_limits_for_pos(PositionRad(0.5)), Radians::ZERO);    // In range
        /// assert_eq!(gear.resolve_limits_for_pos(PositionRad(-4.0)), Radians(-1.0));   // Under the minimum, but less
        /// ```
        fn overwrite_pos_limits(&mut self, min_opt : Option<<Self::Units as UnitSet>::Position>, max_opt : Option<<Self::Units as UnitSet>::Position>) {
            self.__acc_vars_mut(|vars| {
                vars.pos_limits.min = min_opt;
                vars.pos_limits.max = max_opt;
            })
        }
    // 

    /* Movement */
        /// Core function of [Self::drive_rel] & [Self::drive_abs]: For the user to implement, all checks for validity of units considering the bounds etc. will
        /// be done by the main function and are not required again
        fn __drive_rel_core(&mut self, rel_dist : <Self::Units as UnitSet>::Distance, speed : Factor) -> Result<(), ActuatorError<Self::Units>>;

        ///  Core function of [Self::drive_cont]: For the user to implement, all checks for validity of units considering the bounds etc. will
        /// be done by the main function and are not required again
        fn __drive_cont_core(&mut self, speed : Factor, direction : Direction) -> Result<(), ActuatorError<Self::Units>>; 

        /// Moves the component by the relative distance with the given factor of `speed` to the maximum possible. 
        /// [Self::vel_possible()] and a desired speed can be used to calculate a factor
        fn drive_rel(&mut self, rel_dist : <Self::Units as UnitSet>::Distance, speed : Factor) -> Result<(), ActuatorError<Self::Units>> {
            let end_pos = self.position() + rel_dist;
            let res_lim = self.resolve_limits_for_pos(end_pos);

            // Checking for bounds
            if res_lim == <Self::Units as UnitSet>::Distance::ZERO {
                self.__drive_rel_core(rel_dist, speed)
            } else {
                Err(ActuatorError::PosOutOfBounds(end_pos, res_lim))
            }
        }

        /// Moves the component to the absolute position with the given factor of `speed` to the maximum possible. 
        /// [Self::vel_possible()] and a desired speed can be used to calculate a factor
        fn drive_abs(&mut self, end_pos : <Self::Units as UnitSet>::Position, speed : Factor) -> Result<(), ActuatorError<Self::Units>> {
            let res_lim = self.resolve_limits_for_pos(end_pos);

            // Checking for bounds
            if res_lim == <Self::Units as UnitSet>::Distance::ZERO {
                let rel_dist = end_pos - self.position();
                self.__drive_rel_core(rel_dist, speed)
            } else {
                Err(ActuatorError::PosOutOfBounds(end_pos, res_lim))
            }
        }

        /// Start the movement process of the component with the given factor `speed` of the maximum possible velocity.
        /// If a fixed speed is desired, use [Self::vel_possible] to calculate the factor and [Unit::get_direction]
        fn drive_cont_fac(&mut self, speed : Factor, direction : Direction) -> Result<(), ActuatorError<Self::Units>> {
            // Currently no checks required as `speed` and `direction` are safe units
            self.__drive_cont_core(speed, direction)
        }

        /// Start the movement process of the component with the given `speed`, positive values mean [Direction::CW]
        fn drive_cont_speed(&mut self, speed : <Self::Units as UnitSet>::Velocity) -> Result<(), ActuatorError<Self::Units>> {
            let dir = speed.get_direction();

            self.__drive_cont_core(
                Factor::try_new(speed.abs() / self.vel_possible()).ok_or(ActuatorError::InvalidVelocity(speed))?, 
                dir
            )
        }
    /**/
}


/// An advanced actuator allows applying loads that alter the actuators movement
pub trait AdvSyncActuator : SyncActuator {
    /// Function that gets called on load changes, recalculates possible speeds etc. 
    fn update_load_calc(&mut self) -> Result<(), ActuatorError<Self::Units>>;

    /// Returns the general load force, meaning load force that applies in both directions
    /// (most of the time friction)
    fn load_force_gen(&self) -> <Self::Units as UnitSet>::Force {
        self.__acc_vars(|vars| vars.load_force_gen)
    }

    /// Returns the directional load force, meaning load force that is directionally dependent
    fn load_force_dir(&self) -> <Self::Units as UnitSet>::Force {
        self.__acc_vars(|vars| vars.load_force_dir)
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
    /// assert_eq!(PositionRad(2.0), gear.position_for_child(PositionRad(1.0)));
    /// assert_eq!(NewtonMeters(0.1), gear.child().load_force_gen());     // Resistance gets smaller for this gear ratio
    /// ```
    fn apply_gen_force(&mut self, force : <Self::Units as UnitSet>::Force) -> Result<(), ActuatorError<Self::Units>> {
        self.__acc_vars_mut(|vars| vars.load_force_gen = force);
        self.update_load_calc()
    }

    /// Apply a directional load force to the component, slowing down or assisting movements
    /// 
    /// ### Directional load force
    /// 
    /// A directional load force can be imagined like a weight attached to a rope and a pulley on the actuator. 
    /// Downwards the weight assists the motor in movement, upwards it slows the process down.
    /// 
    /// A positive value for this force means the force assists in `CW` direction and slows in `CCW` direction, 
    /// the other way around with negative forces
    fn apply_dir_force(&mut self, force : <Self::Units as UnitSet>::Force) -> Result<(), ActuatorError<Self::Units>> {
        self.__acc_vars_mut(|vars| vars.load_force_dir = force);
        self.update_load_calc()
    }

    /// Returns the inertia applied to the component, see [AdvancedActuator::apply_inertia]
    fn load_inertia(&self) -> <Self::Units as UnitSet>::Inertia {
        self.__acc_vars(|vars| vars.load_inertia)
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
    /// assert_eq!(PositionRad(2.0), gear.position_for_child(PositionRad(1.0)));
    /// assert_eq!(KgMeter2(1.0), gear.child().load_inertia());          // Inertias get smaller with the ratio^2 !
    /// ```
    fn apply_load_inertia(&mut self, inertia : <Self::Units as UnitSet>::Inertia) -> Result<(), ActuatorError<Self::Units>> {
        self.__acc_vars_mut(|vars| vars.load_inertia = inertia);
        self.update_load_calc()
    }
}

/// An actuator that has a defined time to move for a PTP (Point-To-Point) movement
pub trait DefinedActuator : SyncActuator {
    /// The time required to perform a certain PTP (Point-To-Point movement)
    fn ptp_time_for_distance(&self, abs_pos_0 : <Self::Units as UnitSet>::Position, abs_pos_t : <Self::Units as UnitSet>::Position) -> <Self::Units as UnitSet>::Time;
}
// 