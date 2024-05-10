use alloc::sync::Arc;
use atomic_float::AtomicF32;
use syunit::*;

use crate::Setup;

// ####################
// #    SUBMODULES    #
// ####################
    /// A module for async components like a basic DC-motor. These components cannot move certain distances or to absolute positions
    pub mod asyn;

    mod comps;
    pub use comps::{Conveyor, Gear, LinearAxis, StateActuator};

    /// A module for component groups, as they are used in various robots. The components are all sharing the same 
    /// [StepperConfig](crate::data::StepperConfig) and their movements are coordinated. 
    pub mod group;
    pub use group::SyncActuatorGroup;

    /// ALl the parent structures used
    pub mod parent;

    /// Stepper motors and their unique methods and traits
    pub mod stepper;
    pub use stepper::{StepperActuator, Stepper};
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
        fn check(&mut self, gamma : &Arc<AtomicF32>) -> Option<InterruptReason>;
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
        fn intr_reason(&self) -> Option<InterruptReason>;
    }
//

// ######################
// #    SyncActuator    #
// ######################
    // pub enum SyncActuatorError {

    // }


    /// Trait for defining controls and components of synchronous actuators
    /// 
    /// # Parent components
    /// 
    /// Components can have multiple layers, for example take a stepper motor with a geaerbox attached to it. The stepper motor and both combined will be a component, the later having 
    /// the stepper motor component defined as it's parent component. (See [Gear])
    pub trait SyncActuator : Setup {
        // Movement
            /// Moves the component by the relative distance as fast as possible, halts the script until 
            /// the movement is finshed and returns the actual **relative** distance travelled
            fn drive_rel(&mut self, delta : Delta, speed : Factor) -> Result<(), crate::Error>;

            /// Moves the component to the given position as fast as possible, halts the script until the 
            /// movement is finished and returns the actual **relative** distance travelled.
            #[inline]
            fn drive_abs(&mut self, gamma : Gamma, speed : Factor) -> Result<(), crate::Error> {
                let delta = gamma - self.gamma();
                self.drive_rel(delta, speed)
            }
        // 

        // Async
            /// Moves the component by the relative distance as fast as possible
            fn drive_rel_async(&mut self, delta : Delta, speed : Factor) -> Result<(), crate::Error>;

            /// Moves the component to the given position as fast as possible, halts the script until the 
            /// movement is finished and returns the actual **abolute** distance traveled to. 
            #[inline(always)]
            fn drive_abs_async(&mut self, gamma : Gamma, speed : Factor) -> Result<(), crate::Error> {
                let delta = gamma - self.gamma();
                self.drive_rel_async(delta, speed)
            }
            
            /// Drive with a fixed `Velocity`
            fn drive_velocity(&mut self, velocity_tar : Velocity) -> Result<(), crate::Error>;

            /// Halts the thread until the async movement is finished
            /// 
            /// # Errors
            /// 
            /// - Returns an error if the definition has not been overwritten by the component and no parent component is known
            /// - Returns an error if no async movement has been started yet
            fn await_inactive(&mut self) -> Result<(), crate::Error>;
        // 

        // Position
            /// Returns the **absolute** position of the component.
            /// 
            /// ```rust
            /// use syact::prelude::*;
            /// 
            /// // Position of components
            /// const POS : Gamma = Gamma(10.0);
            /// 
            /// // Create a new cylinder (implements SyncComp)
            /// let mut cylinder = LinearAxis::new(
            ///     // Stepper Motor as subcomponent (also implements SyncComp)
            ///     Stepper::new_gen(), 
            /// 0.5);    // Ratio is set to 0.5, which means for each radian the motor moves, the cylinder moves for 0.5 mm
            /// 
            /// cylinder.set_gamma(POS);
            /// 
            /// assert!((cylinder.gamma() - POS).abs() < Delta(0.05));      // Check with small tolerance
            /// ```
            fn gamma(&self) -> Gamma;

            /// Overwrite the current **absolute** position of the component without triggering actual movements. 
            /// 
            /// Be aware that only full steps can be written in distance, meaning that for position comparision a 
            /// small tolerance has to be considered, as the value written won't be the exact gamma value given.
            /// 
            /// ```rust
            /// use syact::prelude::*;
            /// 
            /// // Position of components
            /// const POS : Gamma = Gamma(10.0);
            /// 
            /// // Create a new cylinder (implements SyncComp)
            /// let mut cylinder = LinearAxis::new(
            ///     // Stepper Motor as subcomponent (also implements SyncComp)
            ///     Stepper::new_gen(), 
            /// 0.5);    // Ratio is set to 0.5, which means for each radian the motor moves, the cylinder moves for 0.5 mm
            /// 
            /// cylinder.set_gamma(POS);
            /// 
            /// assert!((cylinder.gamma() - POS).abs() < Delta(0.05));      // Check with small tolerance
            /// ```
            fn set_gamma(&mut self, gamma : Gamma);

            /// Returns the maximum velocity of the component. It can be set using `SyncComp::set_velocity_max()`. 
            /// The component cannot move faster than the velocity  given (valid for all movement operations)
            /// 
            /// # Panics
            /// 
            /// - Panics if no parent component or an override is provided
            fn velocity_max(&self) -> Velocity;

            /// Set the maximum velocity of the component, current maximum velocity  can be access with `SyncComp::velocity_max()`
            /// 
            /// # Panics
            /// 
            /// - Panics if no parent component or an override is provided
            /// - Panics if the velocity  given is higher than the maximum velocity  recommended (e.g. `StepperConst::velocity_max()`)
            fn set_velocity_max(&mut self, velocity_max : Velocity);

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
            /// const LIM_MAX : Gamma = Gamma(1.0);
            /// const LIM_MIN : Gamma = Gamma(-2.0);
            /// 
            /// const LIM_MIN_LOWER : Gamma = Gamma(-3.0);
            /// 
            /// // Create a new gear bearing (implements SyncComp)
            /// let mut gear = Gear::new(
            ///     // Stepper Motor as subcomponent (also implements SyncComp)
            ///     Stepper::new_gen(), 
            /// 0.5);    // Ratio is set to 0.5, which means for each radian the motor moves, the bearing moves for half a radian
            /// 
            /// gear.set_limits(Some(LIM_MIN), Some(LIM_MAX));
            /// 
            /// assert_eq!(gear.limits_for_gamma(Gamma(1.5)), Delta(0.5));     // Over the maximum
            /// assert_eq!(gear.limits_for_gamma(Gamma(0.5)), Delta::ZERO);    // In range
            /// assert_eq!(gear.limits_for_gamma(Gamma(-4.0)), Delta(-2.0));   // Under the minimum
            /// 
            /// gear.set_limits(Some(LIM_MIN_LOWER), None);                // Overwriting only `min` limit
            /// 
            /// assert_eq!(gear.limits_for_gamma(Gamma(1.5)), Delta(0.5));     // Over the maximum
            /// assert_eq!(gear.limits_for_gamma(Gamma(0.5)), Delta::ZERO);    // In range
            /// assert_eq!(gear.limits_for_gamma(Gamma(-4.0)), Delta(-1.0));   // Under the minimum, but less
            /// 
            /// gear.overwrite_limits(Some(LIM_MIN_LOWER), None);              // Overwriting only both limits with [overwrite_limits()]
            /// 
            /// assert_eq!(gear.limits_for_gamma(Gamma(1.5)), Delta::ZERO);    // In range, as the `max` limit has been deleted
            /// assert_eq!(gear.limits_for_gamma(Gamma(0.5)), Delta::ZERO);    // In range
            /// assert_eq!(gear.limits_for_gamma(Gamma(-4.0)), Delta(-1.0));   // Under the minimum, but less
            /// ```
            fn limits_for_gamma(&self, gamma : Gamma) -> Delta;

            /// Sets an endpoint in the current direction by modifying the components limits. For example, when the component is moving
            /// in the positive direction and the endpoint is set, this function will overwrite the current maximum limit with the current
            /// gamma value. The component is then not allowed to move in the current direction anymore. 
            /// 
            /// ```rust
            /// use syact::prelude::*;
            /// 
            /// const GAMMA : Gamma = Gamma(1.0); 
            /// 
            /// // Create a new gear bearing (implements SyncComp)
            /// let mut gear = Gear::new(
            ///     // Stepper Motor as subcomponent (also implements SyncComp)
            ///     Stepper::new_gen(), 
            /// 0.5);    // Ratio is set to 0.5, which means for each radian the motor moves, the bearing moves for half a radian
            /// gear.set_config(StepperConfig::GEN);           // Link component for driving
            /// 
            /// gear.set_gamma(Gamma::ZERO);
            /// 
            /// gear.set_velocity_max(Velocity(5.0));
            /// gear.drive_rel(Delta(-0.2), Factor::MAX).unwrap();    // Drive component in negative direction
            /// 
            /// assert_eq!(gear.dir(), Direction::CCW);
            /// assert!((gear.gamma() - Gamma(-0.2)).abs() < Delta(0.05));     // Check if the movement was correct (with step inaccuracy)
            /// 
            /// gear.set_end(GAMMA);
            /// 
            /// assert_eq!(gear.limits_for_gamma(Gamma(2.0)), Delta::ZERO);     
            /// assert_eq!(gear.limits_for_gamma(Gamma(-2.0)), Delta(-3.0));      
            /// ```
            fn set_end(&mut self, set_gamma : Gamma);

            /// Set the limits for the minimum and maximum angles that the component can reach, note that the limit will 
            /// be converted and transfered to the parent component if defined. 
            /// 
            /// Unlike [SyncComp::overwrite_limits()], this function does not overwrite the current `min` or `max` limits if they
            /// are set to `None`. 
            /// 
            /// ```rust
            /// use syact::prelude::*;
            /// 
            /// // Limits
            /// const LIM_MAX : Gamma = Gamma(1.0);
            /// const LIM_MIN : Gamma = Gamma(-2.0);
            /// 
            /// const LIM_MIN_LOWER : Gamma = Gamma(-3.0);
            /// 
            /// // Create a new gear bearing (implements SyncComp)
            /// let mut gear = Gear::new(
            ///     // Stepper Motor as subcomponent (also implements SyncComp)
            ///     Stepper::new_gen(), 
            /// 0.5);    // Ratio is set to 0.5, which means for each radian the motor moves, the bearing moves for half a radian
            /// 
            /// gear.set_limits(Some(LIM_MIN), Some(LIM_MAX));
            /// 
            /// assert_eq!(gear.limits_for_gamma(Gamma(1.5)), Delta(0.5));     // Over the maximum
            /// assert_eq!(gear.limits_for_gamma(Gamma(0.5)), Delta::ZERO);    // In range
            /// assert_eq!(gear.limits_for_gamma(Gamma(-4.0)), Delta(-2.0));   // Under the minimum
            /// 
            /// gear.set_limits(Some(LIM_MIN_LOWER), None);                // Overwriting only `min` limit
            /// 
            /// assert_eq!(gear.limits_for_gamma(Gamma(1.5)), Delta(0.5));     // Over the maximum
            /// assert_eq!(gear.limits_for_gamma(Gamma(0.5)), Delta::ZERO);    // In range
            /// assert_eq!(gear.limits_for_gamma(Gamma(-4.0)), Delta(-1.0));   // Under the minimum, but less
            /// 
            /// gear.overwrite_limits(Some(LIM_MIN_LOWER), None);              // Overwriting only both limits with [overwrite_limits()]
            /// 
            /// assert_eq!(gear.limits_for_gamma(Gamma(1.5)), Delta::ZERO);    // In range, as the `max` limit has been deleted
            /// assert_eq!(gear.limits_for_gamma(Gamma(0.5)), Delta::ZERO);    // In range
            /// assert_eq!(gear.limits_for_gamma(Gamma(-4.0)), Delta(-1.0));   // Under the minimum, but less
            /// ```
            fn set_limits(&mut self, min : Option<Gamma>, max : Option<Gamma>);

            /// Set the limits for the minimum and maximum angles that the component can reach, note that the limit will 
            /// be converted and transfered to the parent component if this component has one. 
            /// 
            /// The difference to [SyncComp::set_limits()] is that this function **overwrites** the current limits set.
            /// 
            /// ```rust
            /// use syact::prelude::*;
            /// 
            /// // Limits
            /// const LIM_MAX : Gamma = Gamma(1.0);
            /// const LIM_MIN : Gamma = Gamma(-2.0);
            /// 
            /// const LIM_MIN_LOWER : Gamma = Gamma(-3.0);
            /// 
            /// // Create a new gear bearing (implements SyncComp)
            /// let mut gear = Gear::new(
            ///     // Stepper Motor as subcomponent (also implements SyncComp)
            ///     Stepper::new_gen(), 
            /// 0.5);    // Ratio is set to 0.5, which means for each radian the motor moves, the bearing moves for half a radian
            /// 
            /// gear.set_limits(Some(LIM_MIN), Some(LIM_MAX));
            /// 
            /// assert_eq!(gear.limits_for_gamma(Gamma(1.5)), Delta(0.5));     // Over the maximum
            /// assert_eq!(gear.limits_for_gamma(Gamma(0.5)), Delta::ZERO);    // In range
            /// assert_eq!(gear.limits_for_gamma(Gamma(-4.0)), Delta(-2.0));   // Under the minimum
            /// 
            /// gear.set_limits(Some(LIM_MIN_LOWER), None);                // Overwriting only `min` limit
            /// 
            /// assert_eq!(gear.limits_for_gamma(Gamma(1.5)), Delta(0.5));     // Over the maximum
            /// assert_eq!(gear.limits_for_gamma(Gamma(0.5)), Delta::ZERO);    // In range
            /// assert_eq!(gear.limits_for_gamma(Gamma(-4.0)), Delta(-1.0));   // Under the minimum, but less
            /// 
            /// gear.overwrite_limits(Some(LIM_MIN_LOWER), None);              // Overwriting only both limits with [overwrite_limits()]
            /// 
            /// assert_eq!(gear.limits_for_gamma(Gamma(1.5)), Delta::ZERO);    // In range, as the `max` limit has been deleted
            /// assert_eq!(gear.limits_for_gamma(Gamma(0.5)), Delta::ZERO);    // In range
            /// assert_eq!(gear.limits_for_gamma(Gamma(-4.0)), Delta(-1.0));   // Under the minimum, but less
            /// ```
            fn overwrite_limits(&mut self, min : Option<Gamma>, max : Option<Gamma>);
        // 

        // Load calculation
            /// Will always be positive
            fn force_gen(&self) -> Force;

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
            /// // Create a new gear bearing (implements SyncComp)
            /// let mut gear = Gear::new(
            ///     // Stepper Motor as subcomponent (also implements SyncComp)
            ///     Stepper::new_gen(), 
            /// 0.5);    // Ratio is set to 0.5, which means for each radian the motor moves, the bearing moves for half a radian
            /// 
            /// gear.apply_gen_force(FORCE);
            /// 
            /// assert_eq!(Gamma(2.0), gear.gamma_for_child(Gamma(1.0)));
            /// assert_eq!(Force(0.1), gear.child().force_gen());     // Forces get smaller for smaller gears
            /// ```
            fn apply_gen_force(&mut self, force : Force) -> Result<(), crate::Error>;

            /// Value positive in CW direction
            fn apply_dir_force(&mut self, force : Force) -> Result<(), crate::Error>;

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
            /// // Create a new gear bearing (implements SyncComp)
            /// let mut gear = Gear::new(
            ///     // Stepper Motor as subcomponent (also implements SyncComp)
            ///     Stepper::new_gen(), 
            /// 0.5);    // Ratio is set to 0.5, which means for each radian the motor moves, the bearing moves for half a radian
            /// 
            /// // Applies the inertia to the gearbearing component
            /// gear.apply_inertia(INERTIA);
            /// 
            /// assert_eq!(Gamma(2.0), gear.gamma_for_child(Gamma(1.0)));
            /// assert_eq!(Inertia(1.0), gear.child().inertia());
            /// ```
            fn apply_inertia(&mut self, inertia : Inertia);
        // 
    }
//