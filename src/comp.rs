use core::any::type_name;

use crate::ctrl::{Interruptor, InterruptReason};
use crate::Setup;
use crate::data::{CompVars, CompData};
use crate::units::*;

// Submodules
    /// A module for async components like a basic DC-motor. These components cannot move certain distances or to absolute positions
    pub mod asyn;

    mod conveyor;
    pub use conveyor::{Conveyor, StepperConveyor};

    mod cylinder;
    pub use cylinder::{Cylinder, StepperCylinder};

    mod cylinder_triangle;
    pub use cylinder_triangle::{CylinderTriangle, StepperCylTriangle};

    mod gear_joint;
    pub use gear_joint::{Gear, StepperGearJoint};

    /// A module for component groups, as they are used in various robots. The components are all sharing the same 
    /// [CompData](crate::data::CompData) and their movements are coordinated. 
    pub mod group;
    pub use group::SyncCompGroup;

    /// Stepper motors and their unique methods and traits
    pub mod stepper;
    pub use stepper::{StepperComp, StepperMotor};
//

#[cfg(not(feature = "std"))]
#[inline(always)]
fn no_parent() -> crate::Error {
    crate::ErrorKind::NoParent
}

/// Trait for defining controls and components of synchronous actuators
/// 
/// # Parent components
/// 
/// Components can have multiple layers, for example take a stepper motor with a geaerbox attached to it. The stepper motor and both combined will be a component, the later having 
/// the stepper motor component defined as it's parent component. (See [GearJoint])
pub trait SyncComp : Setup {
    // Data
        /// Returns the variables of the component, such as load force, inertia, limits ...
        /// 
        /// ```rust
        /// use syact::{SyncComp, Stepper, StepperConst};
        /// use syact::comp::GearJoint;
        /// use syact::units::*;
        /// 
        /// // Limits
        /// const LIM_MAX : Gamma = Gamma(1.0);
        /// const LIM_MIN : Gamma = Gamma(-2.0);
        /// 
        /// const LIM_MIN_LOWER : Gamma = Gamma(-3.0);
        /// 
        /// // Create a new gear bearing (implements SyncComp)
        /// let mut gear = GearJoint::new(
        ///     // Stepper Motor as subcomponent (also implements SyncComp)
        ///     Stepper::new_gen(), 
        /// 0.5);    // Ratio is set to 0.5, which means for each radian the motor moves, the bearing moves for half a radian
        /// 
        /// ```
        fn vars(&self) -> &CompVars;

        /// Returns the [CompData](crate::data::CompData) of the component
        fn data<'a>(&'a self) -> &'a CompData;
        
        /// Write the [CompData](crate::data::CompData) to the component
        #[inline(always)]
        fn write_data(&mut self, data : crate::data::CompData) {
            if let Some(p_comp) = self.parent_comp_mut() {
                p_comp.write_data(data);
            } else {
                panic!("Provide an implementation for 'write_data' or a parent component")
            }
        }
    //

    // Interruptors
        /// Add an interruptor to the component, often used for measurements or other processes checking the movement
        fn add_interruptor(&mut self, interruptor : Box<dyn Interruptor + Send>) {
            if let Some(p_comp) = self.parent_comp_mut() {
                p_comp.add_interruptor(interruptor)
            } else {
                panic!("Provide an implementation")
            }
        }

        /// Returns the interrupt reason if there is any (returns `None` otherwise)
        /// 
        /// # Note
        /// 
        /// Executing this function will replace the reason with `None`, so if you need to access the value multiple times, you have to store it yourself
        fn intr_reason(&self) -> Option<InterruptReason> {
            if let Some(p_comp) = self.parent_comp() {
                p_comp.intr_reason()
            } else {
                panic!("Provide an implementation")
            }
        }
    // 

    // Parent
        /// Returns a readonly reference to the parent [SyncComp] if it exists, returns `None` otherwise. If not overwritten by the 
        /// trait implementation, this function returns always `None`.
        /// 
        /// # Example
        /// 
        /// A parent component would for example be the stepper motor for a cylinder (See [Cylinder])
        /// 
        /// ```rust
        /// use syact::prelude::*;
        /// 
        /// // Create a new cylinder (implements SyncComp)
        /// let mut cylinder = Cylinder::new(
        ///     // Stepper Motor as subcomponent (also implements SyncComp)
        ///     Stepper::new_gen(), 
        /// 1.0);     
        /// 
        /// // Overwrite the cylinder position
        /// cylinder.write_gamma(Gamma(1.0));
        /// 
        /// // Checks position of parent component (The position of the parent component won't be exactly 1.0, 
        /// // as the stepper motor can only move whole steps)
        /// assert!((cylinder.parent_comp().unwrap().gamma() - Gamma(1.0)).abs() <= cylinder.step_ang());
        /// ```
        #[inline(always)]
        fn parent_comp(&self) -> Option<&dyn SyncComp> {
            None
        }

        /// Returns a mutable reference to the parent [SyncComp] if it exists, returns `None` otherwise
        /// 
        /// # Example
        /// 
        /// A parent component would for example be the stepper motor for a cylinder (See [Cylinder])
        /// 
        /// ```rust
        /// use syact::prelude::*;
        /// 
        /// // Create a new cylinder (implements SyncComp)
        /// let mut cylinder = Cylinder::new(
        ///     // Stepper Motor as subcomponent (also implements SyncComp)
        ///     Stepper::new_gen(), 
        /// 1.0);    
        /// 
        /// // Overwrite position in parent component
        /// cylinder.parent_comp_mut().unwrap().write_gamma(Gamma(1.0));
        /// 
        /// // Check own position (The position of the component won't be exactly 1.0, as the stepper motor can only move whole steps)
        /// assert!((cylinder.parent_comp().unwrap().gamma() - Gamma(1.0)).abs() <= cylinder.step_ang());
        /// ```
        #[inline(always)]
        fn parent_comp_mut(&mut self) -> Option<&mut dyn SyncComp> {
            None
        }

        /// Converts the given **absolute** distance to the **absolute** distance for the parent component
        /// 
        /// # Example
        /// 
        /// When using a gearmotor with a ratio of four (motor movement speed will be reduced to a quater), 
        /// this function will return a parent distance *four times higher* than the input distance.
        /// 
        /// ```rust
        /// use syact::prelude::*;
        /// 
        /// // Create a new cylinder (implements SyncComp)
        /// let mut cylinder = Cylinder::new(
        ///     // Stepper Motor as subcomponent (also implements SyncComp)
        ///     Stepper::new_gen(), 
        /// 2.0);    // Ratio is set to 2.0, which means for each radian the motor moves, the cylinder moves for 2.0 mm
        /// 
        /// assert_eq!(Gamma(1.0), cylinder.gamma_for_parent(Gamma(2.0)));
        /// ```
        #[inline(always)]
        fn gamma_for_parent(&self, this_gamma : Gamma) -> Gamma {
            this_gamma
        }

        /// Converts the given **absolute** distance for the parent component to the **absolute** distance for this component
        /// 
        /// # Example
        /// 
        /// When using a gearmotor with a ratio of four (motor movement speed will be reduced to a quater), 
        /// this function will return a distance *four times higher* than the input parent distance
        /// 
        /// ```rust
        /// use syact::prelude::*;
        /// 
        /// // Create a new cylinder (implements SyncComp)
        /// let mut cylinder = Cylinder::new(
        ///     // Stepper Motor as subcomponent (also implements SyncComp)
        ///     Stepper::new_gen(), 
        /// 2.0);    // Ratio is set to 2.0, which means for each radian the motor moves, the cylinder moves for 2.0 mm
        /// 
        /// assert_eq!(Gamma(2.0), cylinder.gamma_for_this(Gamma(1.0)));
        /// ```
        #[inline(always)]
        fn gamma_for_this(&self, parent_gamma : Gamma) -> Gamma {
            parent_gamma
        }   

        /// Converts the given **absolute** distance for this component to the **absolute** distance for the lowest subcomponent 
        /// of this structure
        /// 
        /// # Example
        /// 
        /// When using a gearmotor with a ratio of four (motor movement speed will be reduced to a quater), 
        /// this function will return a distance *four times higher* than the input parent distance
        /// 
        /// ```rust
        /// use syact::prelude::*;
        /// 
        /// // Create a new cylinder (implements SyncComp)
        /// let mut cylinder = Cylinder::new(
        ///     // Stepper Motor as subcomponent (also implements SyncComp)
        ///     Stepper::new_gen(), 
        /// 2.0);    // Ratio is set to 2.0, which means for each radian the motor moves, the cylinder moves for 2.0 mm
        /// 
        /// assert_eq!(Gamma(1.0), cylinder.abs_parent_gamma(Gamma(2.0)));
        /// ```
        #[inline(always)]
        fn abs_parent_gamma(&self, this_gamma : Gamma) -> Gamma {
            let parent_gamma = self.gamma_for_parent(this_gamma);

            if let Some(comp) = self.parent_comp() {
                comp.abs_parent_gamma(parent_gamma)
            } else {
                parent_gamma
            }
        }

        /// Converts the given **relative** distance [Delta] for this component into the **relative** distance for the parent component
        /// 
        /// # Example
        /// 
        /// When using a cylinder with a ratio of one half (movement speed will be halfed), 
        /// this function will return a [Delta] *twice as high* than the input [Delta]
        /// 
        /// ```rust
        /// use syact::prelude::*;
        /// 
        /// // Position of components
        /// const POS : Gamma = Gamma(10.0);
        /// 
        /// // Create a new cylinder (implements SyncComp)
        /// let mut cylinder = Cylinder::new(
        ///     // Stepper Motor as subcomponent (also implements SyncComp)
        ///     Stepper::new_gen(), 
        /// 0.5);    // Ratio is set to 0.5, which means for each radian the motor moves, the cylinder moves for 0.5 mm
        /// 
        /// cylinder.write_gamma(POS);
        /// 
        /// assert_eq!(Delta(2.0), cylinder.delta_for_parent(Delta(1.0), POS));
        /// ```
        #[inline(always)]
        fn delta_for_parent(&self, this_delta : Delta, this_gamma : Gamma) -> Delta {
            Delta::diff(self.gamma_for_parent(this_gamma), self.gamma_for_parent(this_gamma + this_delta))
        }

        /// Converts the given **relative** distance [Delta] of the parent component into the **relative** distance for the this component
        /// 
        /// # Example
        /// 
        /// When using a cylinder with a ratio of one half (movement speed will be halfed), 
        /// this function will return a [Delta] *half in value* than the input [Delta].
        /// 
        /// ```rust
        /// use syact::prelude::*;
        /// 
        /// // Position of components
        /// const POS : Gamma = Gamma(10.0);
        /// 
        /// // Create a new cylinder (implements SyncComp)
        /// let mut cylinder = Cylinder::new(
        ///     // Stepper Motor as subcomponent (also implements SyncComp)
        ///     Stepper::new_gen(), 
        /// 0.5);    // Ratio is set to 0.5, which means for each radian the motor moves, the cylinder moves for 0.5 mm
        /// 
        /// cylinder.write_gamma(POS);
        /// 
        /// assert_eq!(Delta(1.0), cylinder.delta_for_this(Delta(2.0), POS));
        /// ```
        #[inline(always)]
        fn delta_for_this(&self, parent_delta : Delta, parent_gamma : Gamma) -> Delta {
            Delta::diff(self.gamma_for_this(parent_gamma), self.gamma_for_this(parent_gamma + parent_delta))
        }    

        /// Converts the given velocity into the velocity for the parent component
        #[inline(always)]
        #[allow(unused_variables)]
        fn omega_for_parent(&self, this_omega : Omega, this_gamma : Gamma) -> Omega {
            Omega(self.gamma_for_parent(Gamma(this_omega.into())).into())
        }

        /// Converts the given parent velocity into the velocity for the this component
        #[inline(always)]
        #[allow(unused_variables)]
        fn omega_for_this(&self, parent_omega : Omega, this_gamma : Gamma) -> Omega {
            Omega(self.gamma_for_this(Gamma(parent_omega.into())).into())
        }

        /// Converts the given parent alpha into the alpha for this component
        #[inline]
        fn alpha_for_this(&self, parent_alpha : Alpha) -> Alpha {
            Alpha(self.gamma_for_this(Gamma(parent_alpha.0)).0)
        }   

        /// Converts the given parent force into a force for this component
        #[inline]
        fn force_for_this(&self, parent_force : Force) -> Force {
            Force(self.gamma_for_parent(Gamma(parent_force.0)).0)
        }
    // 

    // Movement
        /// Moves the component by the relative distance as fast as possible, halts the script until 
        /// the movement is finshed and returns the actual **relative** distance travelled
        fn drive_rel(&mut self, mut delta : Delta, speed_f : f32) -> Result<Delta, crate::Error> {
            if (1.0 < speed_f) | (0.0 > speed_f) {
                panic!("Invalid speed factor! {}", speed_f)
            }

            let gamma = self.gamma(); 

            delta = self.delta_for_parent(delta, gamma);

            let res = if let Some(p_comp) = self.parent_comp_mut() {
                p_comp.drive_rel(delta, speed_f)?
            } else {
                #[cfg(feature = "std")]
                panic!("Provide a parent component or an override for this function!");
            };
            
            Ok(self.delta_for_this(res, self.gamma_for_parent(gamma)))
        }

        /// Moves the component to the given position as fast as possible, halts the script until the 
        /// movement is finished and returns the actual **relative** distance travelled.
        fn drive_abs(&mut self, gamma : Gamma, speed_f : f32) -> Result<Delta, crate::Error> {
            let delta = gamma - self.gamma();
            self.drive_rel(delta, speed_f)
        }
    // 

    // Async
        /// Moves the component by the relative distance as fast as possible
        #[inline(always)]
        #[cfg(feature = "std")]
        fn drive_rel_async(&mut self, mut delta : Delta, speed_f : f32) -> Result<(), crate::Error> {
            if (1.0 < speed_f) | (0.0 > speed_f) {
                panic!("Invalid speed factor! {}", speed_f)
            }

            let gamma = self.gamma(); 

            delta = self.delta_for_parent(delta, gamma);

            if let Some(p_comp) = self.parent_comp_mut() {
                p_comp.drive_rel_async(delta, speed_f)
            } else {
                #[cfg(feature = "std")]
                panic!("Provide a parent component or an override for this function!");
            }
        }

        /// Moves the component to the given position as fast as possible, halts the script until the 
        /// movement is finished and returns the actual **abolute** distance traveled to. 
        #[inline(always)]
        #[cfg(feature = "std")]
        fn drive_abs_async(&mut self, gamma : Gamma, speed_f : f32) -> Result<(), crate::Error> {
            let delta = gamma - self.gamma();
            self.drive_rel_async(delta, speed_f)
        }

        fn drive_omega(&mut self, mut omega_tar : Omega) -> Result<(), crate::Error> {
            omega_tar = self.omega_for_parent(omega_tar, self.gamma());

            if let Some(p_comp) = self.parent_comp_mut() {
                p_comp.drive_omega(omega_tar)
            } else {
                #[cfg(feature = "std")]
                panic!("Provide a parent component or an override for this function!");
            }
        }   

        /// Halts the thread until the async movement is finished
        /// 
        /// # Features
        /// 
        /// Only available if the feature "std" is available
        /// 
        /// # Errors
        /// 
        /// - Returns an error if the definition has not been overwritten by the component and no parent component is known
        /// - Returns an error if no async movement has been started yet
        #[inline(always)]
        #[cfg(feature = "std")]
        fn await_inactive(&mut self) -> Result<Delta, crate::Error> {
            let delta = if let Some(p_comp) = self.parent_comp_mut() {
                p_comp.await_inactive()?
            } else {
                #[cfg(feature = "std")]
                panic!("Provide a parent component or an override for this function!");
            }; 

            Ok(self.delta_for_this(delta, self.gamma_for_parent(self.gamma())))
        }
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
        /// let mut cylinder = Cylinder::new(
        ///     // Stepper Motor as subcomponent (also implements SyncComp)
        ///     Stepper::new_gen(), 
        /// 0.5);    // Ratio is set to 0.5, which means for each radian the motor moves, the cylinder moves for 0.5 mm
        /// 
        /// cylinder.write_gamma(POS);
        /// 
        /// assert!((cylinder.gamma() - POS).abs() < Delta(0.05));      // Check with small tolerance
        /// ```
        #[inline(always)]
        fn gamma(&self) -> Gamma {
            let parent_len = if let Some(p_comp) = self.parent_comp() {
                p_comp.gamma()
            } else {
                #[cfg(feature = "std")]
                panic!("Provide a parent component or an override for this function!");
            }; 

            self.gamma_for_this(parent_len)
        }

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
        /// let mut cylinder = Cylinder::new(
        ///     // Stepper Motor as subcomponent (also implements SyncComp)
        ///     Stepper::new_gen(), 
        /// 0.5);    // Ratio is set to 0.5, which means for each radian the motor moves, the cylinder moves for 0.5 mm
        /// 
        /// cylinder.write_gamma(POS);
        /// 
        /// assert!((cylinder.gamma() - POS).abs() < Delta(0.05));      // Check with small tolerance
        /// ```
        #[inline(always)]
        fn write_gamma(&mut self, mut gamma : Gamma) {
            gamma = self.gamma_for_parent(gamma);

            if let Some(p_comp) = self.parent_comp_mut() {
                p_comp.write_gamma(gamma);
            } else {
                #[cfg(feature = "std")]
                panic!("Provide a parent component or an override for this function!");
            }
        }

        /// Returns the maximum velocity of the component. It can be set using `SyncComp::set_omega_max()`. 
        /// The component cannot move faster than the omega given (valid for all movement operations)
        /// 
        /// # Panics
        /// 
        /// - Panics if no parent component or an override is provided
        #[inline]
        fn omega_max(&self) -> Omega {
            let parent_omega = if let Some(p_comp) = self.parent_comp() {
                p_comp.omega_max()
            } else {
                #[cfg(feature = "std")]
                panic!("Provide a parent component or an override for this function!");
            }; 

            self.omega_for_this(parent_omega, self.gamma())
        }

        /// Set the maximum velocity of the component, current maximum omega can be access with `SyncComp::omega_max()`
        /// 
        /// # Panics
        /// 
        /// - Panics if no parent component or an override is provided
        /// - Panics if the omega given is higher than the maximum omega recommended (e.g. `StepperConst::omega_max()`)
        #[inline]
        fn set_omega_max(&mut self, mut omega_max : Omega) {
            omega_max = self.omega_for_parent(omega_max, self.gamma());

            if let Some(p_comp) = self.parent_comp_mut() {
                p_comp.set_omega_max(omega_max);
            } else {
                #[cfg(feature = "std")]
                panic!("Provide a parent component or an override for this function!");
            }
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
        /// const LIM_MAX : Gamma = Gamma(1.0);
        /// const LIM_MIN : Gamma = Gamma(-2.0);
        /// 
        /// const LIM_MIN_LOWER : Gamma = Gamma(-3.0);
        /// 
        /// // Create a new gear bearing (implements SyncComp)
        /// let mut gear = GearJoint::new(
        ///     // Stepper Motor as subcomponent (also implements SyncComp)
        ///     Stepper::new_gen(), 
        /// 0.5);    // Ratio is set to 0.5, which means for each radian the motor moves, the bearing moves for half a radian
        /// 
        /// gear.set_limit(Some(LIM_MIN), Some(LIM_MAX));
        /// 
        /// assert_eq!(gear.lim_for_gamma(Gamma(1.5)), Delta(0.5));     // Over the maximum
        /// assert_eq!(gear.lim_for_gamma(Gamma(0.5)), Delta::ZERO);    // In range
        /// assert_eq!(gear.lim_for_gamma(Gamma(-4.0)), Delta(-2.0));   // Under the minimum
        /// 
        /// gear.set_limit(Some(LIM_MIN_LOWER), None);                // Overwriting only `min` limit
        /// 
        /// assert_eq!(gear.lim_for_gamma(Gamma(1.5)), Delta(0.5));     // Over the maximum
        /// assert_eq!(gear.lim_for_gamma(Gamma(0.5)), Delta::ZERO);    // In range
        /// assert_eq!(gear.lim_for_gamma(Gamma(-4.0)), Delta(-1.0));   // Under the minimum, but less
        /// 
        /// gear.reset_limit(Some(LIM_MIN_LOWER), None);              // Overwriting only both limits with [reset_limit()]
        /// 
        /// assert_eq!(gear.lim_for_gamma(Gamma(1.5)), Delta::ZERO);    // In range, as the `max` limit has been deleted
        /// assert_eq!(gear.lim_for_gamma(Gamma(0.5)), Delta::ZERO);    // In range
        /// assert_eq!(gear.lim_for_gamma(Gamma(-4.0)), Delta(-1.0));   // Under the minimum, but less
        /// ```
        #[inline(always)]
        fn lim_for_gamma(&self, mut gamma : Gamma) -> Delta {
            gamma = self.gamma_for_parent(gamma);

            let delta = if let Some(p_comp) = self.parent_comp() {
                p_comp.lim_for_gamma(gamma)
            } else {
                #[cfg(feature = "std")]
                panic!("Provide a parent component or an override for this function!");
            };

            self.delta_for_this(delta, gamma)
        }

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
        /// let mut gear = GearJoint::new(
        ///     // Stepper Motor as subcomponent (also implements SyncComp)
        ///     Stepper::new_gen(), 
        /// 0.5);    // Ratio is set to 0.5, which means for each radian the motor moves, the bearing moves for half a radian
        /// gear.write_data(CompData::GEN);           // Link component for driving
        /// 
        /// gear.set_omega_max(Omega(5.0));
        /// gear.drive_rel(Delta(-0.1), 1.0).unwrap();    // Drive component in negative direction
        /// 
        /// gear.set_end(GAMMA);
        /// 
        /// assert_eq!(gear.lim_for_gamma(Gamma(2.0)), Delta::ZERO);     
        /// assert_eq!(gear.lim_for_gamma(Gamma(-2.0)), Delta(-3.0));      
        /// ```
        #[inline(always)]
        fn set_end(&mut self, mut set_gamma : Gamma) {
            set_gamma = self.gamma_for_parent(set_gamma);

            if let Some(p_comp) = self.parent_comp_mut() {
                p_comp.set_end(set_gamma)
            } else {
                #[cfg(feature = "std")]
                panic!("Provide a parent component or an override for this function!");
            }
        }

        /// Set the limits for the minimum and maximum angles that the component can reach, note that the limit will 
        /// be converted and transfered to the parent component if defined. 
        /// 
        /// Unlike [SyncComp::reset_limit()], this function does not overwrite the current `min` or `max` limits if they
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
        /// let mut gear = GearJoint::new(
        ///     // Stepper Motor as subcomponent (also implements SyncComp)
        ///     Stepper::new_gen(), 
        /// 0.5);    // Ratio is set to 0.5, which means for each radian the motor moves, the bearing moves for half a radian
        /// 
        /// gear.set_limit(Some(LIM_MIN), Some(LIM_MAX));
        /// 
        /// assert_eq!(gear.lim_for_gamma(Gamma(1.5)), Delta(0.5));     // Over the maximum
        /// assert_eq!(gear.lim_for_gamma(Gamma(0.5)), Delta::ZERO);    // In range
        /// assert_eq!(gear.lim_for_gamma(Gamma(-4.0)), Delta(-2.0));   // Under the minimum
        /// 
        /// gear.set_limit(Some(LIM_MIN_LOWER), None);                // Overwriting only `min` limit
        /// 
        /// assert_eq!(gear.lim_for_gamma(Gamma(1.5)), Delta(0.5));     // Over the maximum
        /// assert_eq!(gear.lim_for_gamma(Gamma(0.5)), Delta::ZERO);    // In range
        /// assert_eq!(gear.lim_for_gamma(Gamma(-4.0)), Delta(-1.0));   // Under the minimum, but less
        /// 
        /// gear.reset_limit(Some(LIM_MIN_LOWER), None);              // Overwriting only both limits with [reset_limit()]
        /// 
        /// assert_eq!(gear.lim_for_gamma(Gamma(1.5)), Delta::ZERO);    // In range, as the `max` limit has been deleted
        /// assert_eq!(gear.lim_for_gamma(Gamma(0.5)), Delta::ZERO);    // In range
        /// assert_eq!(gear.lim_for_gamma(Gamma(-4.0)), Delta(-1.0));   // Under the minimum, but less
        /// ```
        fn set_limit(&mut self, mut min : Option<Gamma>, mut max : Option<Gamma>) {
            min = match min {   // Update the value with parent component gammas
                Some(min) => Some(self.gamma_for_parent(min)),
                None => None
            }; 

            max = match max {   // Update the value with parent component gammas
                Some(max) => Some(self.gamma_for_parent(max)),
                None => None
            };

            if let Some(p_comp) = self.parent_comp_mut() {
                p_comp.set_limit(min, max)      // If the parent component exists
            } else {
                #[cfg(feature = "std")]
                panic!("Provide a parent component or an override for this function!");
            }
        }

        /// Set the limits for the minimum and maximum angles that the component can reach, note that the limit will 
        /// be converted and transfered to the parent component if this component has one. 
        /// 
        /// The difference to [SyncComp::set_limit()] is that this function **overwrites** the current limits set.
        /// 
        /// ```rust
        /// use syact::{SyncComp, Stepper, StepperConst};
        /// use syact::comp::GearJoint;
        /// use syact::units::*;
        /// 
        /// // Limits
        /// const LIM_MAX : Gamma = Gamma(1.0);
        /// const LIM_MIN : Gamma = Gamma(-2.0);
        /// 
        /// const LIM_MIN_LOWER : Gamma = Gamma(-3.0);
        /// 
        /// // Create a new gear bearing (implements SyncComp)
        /// let mut gear = GearJoint::new(
        ///     // Stepper Motor as subcomponent (also implements SyncComp)
        ///     Stepper::new_gen(), 
        /// 0.5);    // Ratio is set to 0.5, which means for each radian the motor moves, the bearing moves for half a radian
        /// 
        /// gear.set_limit(Some(LIM_MIN), Some(LIM_MAX));
        /// 
        /// assert_eq!(gear.lim_for_gamma(Gamma(1.5)), Delta(0.5));     // Over the maximum
        /// assert_eq!(gear.lim_for_gamma(Gamma(0.5)), Delta::ZERO);    // In range
        /// assert_eq!(gear.lim_for_gamma(Gamma(-4.0)), Delta(-2.0));   // Under the minimum
        /// 
        /// gear.set_limit(Some(LIM_MIN_LOWER), None);                // Overwriting only `min` limit
        /// 
        /// assert_eq!(gear.lim_for_gamma(Gamma(1.5)), Delta(0.5));     // Over the maximum
        /// assert_eq!(gear.lim_for_gamma(Gamma(0.5)), Delta::ZERO);    // In range
        /// assert_eq!(gear.lim_for_gamma(Gamma(-4.0)), Delta(-1.0));   // Under the minimum, but less
        /// 
        /// gear.reset_limit(Some(LIM_MIN_LOWER), None);              // Overwriting only both limits with [reset_limit()]
        /// 
        /// assert_eq!(gear.lim_for_gamma(Gamma(1.5)), Delta::ZERO);    // In range, as the `max` limit has been deleted
        /// assert_eq!(gear.lim_for_gamma(Gamma(0.5)), Delta::ZERO);    // In range
        /// assert_eq!(gear.lim_for_gamma(Gamma(-4.0)), Delta(-1.0));   // Under the minimum, but less
        /// ```
        fn reset_limit(&mut self, mut min : Option<Gamma>, mut max : Option<Gamma>) {
            min = match min {
                Some(min) => Some(self.gamma_for_parent(min)),
                None => None
            }; 

            max = match max {
                Some(max) => Some(self.gamma_for_parent(max)),
                None => None
            };

            if let Some(p_comp) = self.parent_comp_mut() {
                p_comp.reset_limit(min, max)
            } else {
                #[cfg(feature = "std")]
                panic!("Provide a parent component or an override for this function!");
            }
        }
    // 

    // Load calculation
        /// Apply a load force to the component, slowing down movements 
        /// 
        /// ```rust
        /// use syact::{SyncComp, Stepper, StepperConst};
        /// use syact::comp::GearJoint;
        /// use syact::units::*;
        /// 
        /// // Force to act upon the component
        /// const FORCE : Force = Force(0.2);
        /// 
        /// // Create a new gear bearing (implements SyncComp)
        /// let mut gear = GearJoint::new(
        ///     // Stepper Motor as subcomponent (also implements SyncComp)
        ///     Stepper::new_gen(), 
        /// 0.5);    // Ratio is set to 0.5, which means for each radian the motor moves, the bearing moves for half a radian
        /// 
        /// gear.apply_force(FORCE);
        /// 
        /// assert_eq!(Gamma(2.0), gear.gamma_for_parent(Gamma(1.0)));
        /// assert_eq!(Force(0.1), gear.parent_comp().unwrap().vars().t_load);
        /// ```
        #[inline(always)]
        fn apply_force(&mut self, mut force : Force) { // TODO: Add overload protection
            force = Force(self.gamma_for_this(Gamma(force.0)).0);

            if let Some(p_comp) = self.parent_comp_mut() {
                p_comp.apply_force(force);
            } else {
                #[cfg(feature = "std")]
                panic!("Provide a parent component or an override for this function!");
            }
        }
        
        /// Apply a load inertia to the component, slowing down movements
        /// 
        /// # Panics
        /// 
        /// Panics if no parent component or override of the function has been provided.
        /// 
        /// ```rust
        /// use syact::{SyncComp, Stepper, StepperConst};
        /// use syact::comp::GearJoint;
        /// use syact::units::*;
        /// 
        /// // Inertia to act upon the component
        /// const INERTIA : Inertia = Inertia(4.0);
        /// 
        /// // Create a new gear bearing (implements SyncComp)
        /// let mut gear = GearJoint::new(
        ///     // Stepper Motor as subcomponent (also implements SyncComp)
        ///     Stepper::new_gen(), 
        /// 0.5);    // Ratio is set to 0.5, which means for each radian the motor moves, the bearing moves for half a radian
        /// 
        /// // Applies the inertia to the gearbearing component
        /// gear.apply_inertia(INERTIA);
        /// 
        /// assert_eq!(Gamma(2.0), gear.gamma_for_parent(Gamma(1.0)));
        /// assert_eq!(Inertia(1.0), gear.parent_comp().unwrap().vars().j_load);
        /// ```
        #[inline(always)]
        fn apply_inertia(&mut self, mut inertia : Inertia) {
            inertia = Inertia(self.gamma_for_this(self.gamma_for_this(Gamma(inertia.0))).0);

            if let Some(p_comp) = self.parent_comp_mut() {
                p_comp.apply_inertia(inertia);
            } else {
                #[cfg(feature = "std")]
                panic!("Provide a parent component or an override for this function!");
            }
        }

        /// Applies a bend factor to the given component, bending down its movements. 
        /// 
        /// # Panics
        /// 
        /// The function panics if the factor given is bigger than 1.0, equal to 0.0 or negative. Also it panics if neither the function 
        /// definition has not been overwritten nor has a parent component been provied.
        #[inline(always)]
        fn apply_bend_f(&mut self, f_bend : f32) {
            if let Some(p_comp) = self.parent_comp_mut() {
                p_comp.apply_bend_f(f_bend);
            } else {
                #[cfg(feature = "std")]
                panic!("Provide a parent component or an override for this function!");
            }
        }
    // 
}

// Implementations

impl dyn SyncComp {
    /// Returns the type name of the component as [String]. Used for configuration file parsing.
    #[inline(always)]
    pub fn get_type_name(&self) -> &str {
        type_name::<Self>()
    }
}

impl<T : AsMut<dyn SyncComp>> Setup for T {
    fn setup(&mut self) -> Result<(), crate::Error> {
        self.as_mut().setup()
    }
}

impl<T : AsRef<dyn SyncComp> + AsMut<dyn SyncComp>> SyncComp for T {
    // Data
        fn vars<'a>(&'a self) -> &'a crate::prelude::CompVars {
            self.as_ref().vars()
        }

        fn data<'a>(&'a self) -> &'a crate::CompData {
            self.as_ref().data()
        }
    // 

    fn write_data(&mut self, data : crate::data::CompData) {
        self.as_mut().write_data(data)
    }

    fn parent_comp(&self) -> Option<&dyn SyncComp> {
        self.as_ref().parent_comp()
    }

    fn parent_comp_mut(&mut self) -> Option<&mut dyn SyncComp> {
        self.as_mut().parent_comp_mut()
    }

    fn gamma_for_parent(&self, this_gamma : Gamma) -> Gamma {
        self.as_ref().gamma_for_parent(this_gamma)
    }

    fn gamma_for_this(&self, parent_gamma : Gamma) -> Gamma {
        self.as_ref().gamma_for_this(parent_gamma)
    }

    fn abs_parent_gamma(&self, this_gamma : Gamma) -> Gamma {
        self.as_ref().abs_parent_gamma(this_gamma)
    }

    fn delta_for_parent(&self, this_delta : Delta, this_gamma : Gamma) -> Delta {
        self.as_ref().delta_for_parent(this_delta, this_gamma)
    }

    fn delta_for_this(&self, parent_delta : Delta, parent_gamma : Gamma) -> Delta {
        self.as_ref().delta_for_this(parent_delta, parent_gamma)
    }

    fn omega_for_parent(&self, this_omega : Omega, this_gamma : Gamma) -> Omega {
        self.as_ref().omega_for_parent(this_omega, this_gamma)
    }

    fn omega_for_this(&self, parent_omega : Omega, this_gamma : Gamma) -> Omega {
        self.as_ref().omega_for_this(parent_omega, this_gamma)
    }

    fn drive_rel(&mut self, delta : Delta, speed_f : f32) -> Result<Delta, crate::Error> {
        self.as_mut().drive_rel(delta, speed_f)
    }

    fn drive_abs(&mut self, gamma : Gamma, speed_f : f32) -> Result<Delta, crate::Error> {
        self.as_mut().drive_abs(gamma, speed_f)
    }

    fn drive_rel_async(&mut self, delta : Delta, speed_f : f32) -> Result<(), crate::Error> {
        self.as_mut().drive_rel_async(delta, speed_f)
    }

    fn drive_abs_async(&mut self, gamma : Gamma, speed_f : f32) -> Result<(), crate::Error> {
        self.as_mut().drive_abs_async(gamma, speed_f)
    }

    fn await_inactive(&mut self) -> Result<Delta, crate::Error> {
        self.as_mut().await_inactive()
    }

    fn gamma(&self) -> Gamma {
        self.as_ref().gamma()
    }

    fn write_gamma(&mut self, gamma : Gamma) {
        self.as_mut().write_gamma(gamma)
    }

    fn omega_max(&self) -> Omega {
        self.as_ref().omega_max()
    }

    fn set_omega_max(&mut self, omega_max : Omega) {
        self.as_mut().set_omega_max(omega_max)
    }

    fn lim_for_gamma(&self, gamma : Gamma) -> Delta {
        self.as_ref().lim_for_gamma(gamma)
    }

    fn set_end(&mut self, set_gamma : Gamma) {
        self.as_mut().set_end(set_gamma)
    }

    fn set_limit(&mut self, min : Option<Gamma>, max : Option<Gamma>) {
        self.as_mut().set_limit(min, max)
    }

    fn reset_limit(&mut self, min : Option<Gamma>, max : Option<Gamma>) {
        self.as_mut().reset_limit(min, max)
    }

    fn apply_force(&mut self, force : Force) { 
        self.as_mut().apply_force(force)
    }

    fn apply_inertia(&mut self, inertia : Inertia) {
        self.as_mut().apply_inertia(inertia)
    }

    fn apply_bend_f(&mut self, f_bend : f32) {
        self.as_mut().apply_bend_f(f_bend)
    }
}
