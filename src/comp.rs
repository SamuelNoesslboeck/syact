use core::any::type_name;

use crate::{Setup, StepperConst};
use crate::data::{CompVars, LinkedData};
use crate::units::*;

// Submodules
/// A module for async components like a basic DC-motor. These components cannot move certain distances or to absolute positions
pub mod asyn;

mod cylinder;
pub use cylinder::Cylinder;

mod cylinder_triangle;
pub use cylinder_triangle::CylinderTriangle;

mod gear_joint;
pub use gear_joint::GearJoint;

/// A module for component groups, as they are used in various robots. The components are all sharing the same 
/// [LinkedData](crate::data::LinkedData) and their movements are coordinated. 
pub mod group;
pub use group::SyncCompGroup;

#[doc = "../docs/tools.md"]
pub mod tool;
pub use tool::Tool;
//

#[cfg(not(feature = "std"))]
#[inline(always)]
fn no_super() -> crate::Error {
    crate::ErrorKind::NoSuper
}

/// Trait for defining controls and components of synchronous actuators
/// 
/// # Super components
/// 
/// Components can have multiple layers, for example take a stepper motor with a geaerbox attached to it. The stepper motor and both combined will be a component, the later having 
/// the stepper motor component defined as it's super component. (See [GearJoint])
pub trait SyncComp : crate::meas::SimpleMeas + core::fmt::Debug + Setup {
    // Data
        /// Returns the constants the stepper motor used by the component
        fn consts<'a>(&'a self) -> &'a StepperConst;

        /// Returns the variables of the component, such as load force, inertia, limits ...
        /// 
        /// ```rust
        /// use stepper_lib::{SyncComp, StepperCtrl, StepperConst};
        /// use stepper_lib::comp::GearJoint;
        /// use stepper_lib::units::*;
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
        ///     StepperCtrl::new_sim(StepperConst::GEN), 
        /// 0.5);    // Ratio is set to 0.5, which means for each radian the motor moves, the bearing moves for half a radian
        /// 
        /// ```
        fn vars<'a>(&'a self) -> &'a CompVars;

        /// Returns the [LinkedData](crate::data::LinkedData) of the component
        fn link<'a>(&'a self) -> &'a LinkedData;
        
        /// Write the [LinkedData](crate::data::LinkedData) to the component
        #[inline(always)]
        fn write_link(&mut self, lk : crate::data::LinkedData) {
            if let Some(s_comp) = self.super_comp_mut() {
                s_comp.write_link(lk);
            }
        }

        // JSON I/O 
        /// Get the *JSON* data of the current component as [serde_json::Value]
        /// 
        /// # Feature
        /// 
        /// Only available when the "std"-feature is enabled
        #[cfg(feature = "std")]
        fn to_json(&self) -> Result<serde_json::Value, serde_json::Error>;
    // 

    // Super
        /// Returns a readonly reference to the super [SyncComp] if it exists, returns `None` otherwise. If not overwritten by the 
        /// trait implementation, this function returns always `None`.
        /// 
        /// # Example
        /// 
        /// A super component would for example be the stepper motor for a cylinder (See [Cylinder])
        /// 
        /// ```rust
        /// use stepper_lib::{SyncComp, StepperCtrl, StepperConst};
        /// use stepper_lib::comp::Cylinder;
        /// use stepper_lib::units::*;
        /// 
        /// // Create a new cylinder (implements SyncComp)
        /// let mut cylinder = Cylinder::new(
        ///     // Stepper Motor as subcomponent (also implements SyncComp)
        ///     StepperCtrl::new_sim(StepperConst::GEN), 
        /// 1.0);     
        /// 
        /// // Overwrite the cylinder position
        /// cylinder.write_gamma(Gamma(1.0));
        /// 
        /// // Checks position of super component (The position of the super component won't be exactly 1.0, 
        /// // as the stepper motor can only move whole steps)
        /// assert!((cylinder.super_comp().unwrap().gamma() - Gamma(1.0)).abs() <= StepperConst::GEN.step_ang());
        /// ```
        #[inline(always)]
        fn super_comp(&self) -> Option<&dyn SyncComp> {
            None
        }

        /// Returns a mutable reference to the super [SyncComp] if it exists, returns `None` otherwise
        /// 
        /// # Example
        /// 
        /// A super component would for example be the stepper motor for a cylinder (See [Cylinder])
        /// 
        /// ```rust
        /// use stepper_lib::{SyncComp, StepperCtrl, StepperConst};
        /// use stepper_lib::comp::Cylinder;
        /// use stepper_lib::units::*;
        /// 
        /// // Create a new cylinder (implements SyncComp)
        /// let mut cylinder = Cylinder::new(
        ///     // Stepper Motor as subcomponent (also implements SyncComp)
        ///     StepperCtrl::new_sim(StepperConst::GEN), 
        /// 1.0);    
        /// 
        /// // Overwrite position in super component
        /// cylinder.super_comp_mut().unwrap().write_gamma(Gamma(1.0));
        /// 
        /// // Check own position (The position of the component won't be exactly 1.0, as the stepper motor can only move whole steps)
        /// assert!((cylinder.super_comp().unwrap().gamma() - Gamma(1.0)).abs() <= StepperConst::GEN.step_ang());
        /// ```
        #[inline(always)]
        fn super_comp_mut(&mut self) -> Option<&mut dyn SyncComp> {
            None
        }

        /// Converts the given **absolute** distance to the **absolute** distance for the super component
        /// 
        /// # Example
        /// 
        /// When using a gearmotor with a ratio of four (motor movement speed will be reduced to a quater), 
        /// this function will return a super distance *four times higher* than the input distance.
        /// 
        /// ```rust
        /// use stepper_lib::{SyncComp, StepperCtrl, StepperConst};
        /// use stepper_lib::comp::Cylinder;
        /// use stepper_lib::units::*;
        /// 
        /// 
        /// // Create a new cylinder (implements SyncComp)
        /// let mut cylinder = Cylinder::new(
        ///     // Stepper Motor as subcomponent (also implements SyncComp)
        ///     StepperCtrl::new_sim(StepperConst::GEN), 
        /// 2.0);    // Ratio is set to 2.0, which means for each radian the motor moves, the cylinder moves for 2.0 mm
        /// 
        /// assert_eq!(Gamma(1.0), cylinder.gamma_for_super(Gamma(2.0)));
        /// ```
        #[inline(always)]
        fn gamma_for_super(&self, this_gamma : Gamma) -> Gamma {
            this_gamma
        }

        /// Converts the given **absolute** distance for the super component to the **absolute** distance for this component
        /// 
        /// # Example
        /// 
        /// When using a gearmotor with a ratio of four (motor movement speed will be reduced to a quater), 
        /// this function will return a distance *four times higher* than the input super distance
        /// 
        /// ```rust
        /// use stepper_lib::{SyncComp, StepperCtrl, StepperConst};
        /// use stepper_lib::comp::Cylinder;
        /// use stepper_lib::units::*;
        /// 
        /// // Create a new cylinder (implements SyncComp)
        /// let mut cylinder = Cylinder::new(
        ///     // Stepper Motor as subcomponent (also implements SyncComp)
        ///     StepperCtrl::new_sim(StepperConst::GEN), 
        /// 2.0);    // Ratio is set to 2.0, which means for each radian the motor moves, the cylinder moves for 2.0 mm
        /// 
        /// assert_eq!(Gamma(2.0), cylinder.gamma_for_this(Gamma(1.0)));
        /// ```
        #[inline(always)]
        fn gamma_for_this(&self, super_gamma : Gamma) -> Gamma {
            super_gamma
        }   

        /// Converts the given **absolute** distance for the super component to the **absolute** distance for this component
        /// 
        /// # Example
        /// 
        /// When using a gearmotor with a ratio of four (motor movement speed will be reduced to a quater), 
        /// this function will return a distance *four times higher* than the input super distance
        /// 
        /// ```rust
        /// use stepper_lib::{SyncComp, StepperCtrl, StepperConst};
        /// use stepper_lib::comp::Cylinder;
        /// use stepper_lib::units::*;
        /// 
        /// // Create a new cylinder (implements SyncComp)
        /// let mut cylinder = Cylinder::new(
        ///     // Stepper Motor as subcomponent (also implements SyncComp)
        ///     StepperCtrl::new_sim(StepperConst::GEN), 
        /// 2.0);    // Ratio is set to 2.0, which means for each radian the motor moves, the cylinder moves for 2.0 mm
        /// 
        /// assert_eq!(Gamma(1.0), cylinder.abs_super_gamma(Gamma(2.0)));
        /// ```
        #[inline(always)]
        fn abs_super_gamma(&self, this_gamma : Gamma) -> Gamma {
            let super_gamma = self.gamma_for_super(this_gamma);

            if let Some(comp) = self.super_comp() {
                comp.abs_super_gamma(super_gamma)
            } else {
                super_gamma
            }
        }

        /// Converts the given **relative** distance [Delta] for this component into the **relative** distance for the super component
        /// 
        /// # Example
        /// 
        /// When using a cylinder with a ratio of one half (movement speed will be halfed), 
        /// this function will return a [Delta] *twice as high* than the input [Delta]
        /// 
        /// ```rust
        /// use stepper_lib::{SyncComp, StepperCtrl, StepperConst};
        /// use stepper_lib::comp::Cylinder;
        /// use stepper_lib::units::*;
        /// 
        /// // Position of components
        /// const POS : Gamma = Gamma(10.0);
        /// 
        /// // Create a new cylinder (implements SyncComp)
        /// let mut cylinder = Cylinder::new(
        ///     // Stepper Motor as subcomponent (also implements SyncComp)
        ///     StepperCtrl::new_sim(StepperConst::GEN), 
        /// 0.5);    // Ratio is set to 0.5, which means for each radian the motor moves, the cylinder moves for 0.5 mm
        /// 
        /// cylinder.write_gamma(POS);
        /// 
        /// assert_eq!(Delta(2.0), cylinder.delta_for_super(Delta(1.0), POS));
        /// ```
        #[inline(always)]
        fn delta_for_super(&self, this_delta : Delta, this_gamma : Gamma) -> Delta {
            Delta::diff(self.gamma_for_super(this_gamma), self.gamma_for_super(this_gamma + this_delta))
        }

        /// Converts the given **relative** distance [Delta] of the super component into the **relative** distance for the this component
        /// 
        /// # Example
        /// 
        /// When using a cylinder with a ratio of one half (movement speed will be halfed), 
        /// this function will return a [Delta] *half in value* than the input [Delta].
        /// 
        /// ```rust
        /// use stepper_lib::{SyncComp, StepperCtrl, StepperConst};
        /// use stepper_lib::comp::Cylinder;
        /// use stepper_lib::units::*;
        /// 
        /// // Position of components
        /// const POS : Gamma = Gamma(10.0);
        /// 
        /// // Create a new cylinder (implements SyncComp)
        /// let mut cylinder = Cylinder::new(
        ///     // Stepper Motor as subcomponent (also implements SyncComp)
        ///     StepperCtrl::new_sim(StepperConst::GEN), 
        /// 0.5);    // Ratio is set to 0.5, which means for each radian the motor moves, the cylinder moves for 0.5 mm
        /// 
        /// cylinder.write_gamma(POS);
        /// 
        /// assert_eq!(Delta(1.0), cylinder.delta_for_this(Delta(2.0), POS));
        /// ```
        #[inline(always)]
        fn delta_for_this(&self, super_delta : Delta, super_gamma : Gamma) -> Delta {
            Delta::diff(self.gamma_for_this(super_gamma), self.gamma_for_this(super_gamma + super_delta))
        }    

        /// Converts the given velocity into the velocity for the super component
        #[inline(always)]
        #[allow(unused_variables)]
        fn omega_for_super(&self, this_omega : Omega, this_gamma : Gamma) -> Omega {
            Omega(self.gamma_for_super(Gamma(this_omega.into())).into())
        }

        /// Converts the given super velocity into the velocity for the this component
        #[inline(always)]
        #[allow(unused_variables)]
        fn omega_for_this(&self, super_omega : Omega, this_gamma : Gamma) -> Omega {
            Omega(self.gamma_for_this(Gamma(super_omega.into())).into())
        }

        /// Converts the given acceleration into the acceleration for the super component
        #[inline(always)]
        #[allow(unused_variables)]
        fn alpha_for_super(&self, this_alpha : Alpha, this_gamma : Gamma) -> Alpha {
            Alpha(self.gamma_for_super(Gamma(this_alpha.into())).into())
        }

        /// Converts the given super acceleration into the acceleration for this component
        #[inline(always)]
        #[allow(unused_variables)]
        fn alpha_for_this(&self, super_alpha : Alpha, super_gamma : Gamma) -> Alpha {
            Alpha(self.gamma_for_this(Gamma(super_alpha.into())).into())
        }
    // 

    // Movement
        /// Moves the component by the relative distance as fast as possible, halts the script until 
        /// the movement is finshed and returns the actual **relative** distance travelled
        fn drive_rel(&mut self, mut delta : Delta, mut omega : Omega) -> Result<Delta, crate::Error> {
            let gamma = self.gamma(); 

            delta = self.delta_for_super(delta, gamma);
            omega = self.omega_for_super(omega, gamma);

            let res = if let Some(s_comp) = self.super_comp_mut() {
                s_comp.drive_rel(delta, omega)?
            } else {
                #[cfg(feature = "std")]
                panic!("Provide a super component or an override for this component!");
            };
            
            Ok(self.delta_for_this(res, self.gamma_for_super(gamma)))
        }

        /// Moves the component to the given position as fast as possible, halts the script until the 
        /// movement is finished and returns the actual **relative** distance travelled.
        fn drive_abs(&mut self, mut gamma : Gamma, mut omega : Omega) -> Result<Delta, crate::Error> {
            omega = self.omega_for_super(omega, gamma);
            gamma = self.gamma_for_super(gamma);

            let res = if let Some(s_comp) = self.super_comp_mut() {
                s_comp.drive_abs(gamma, omega)?
            } else {
                #[cfg(feature = "std")]
                panic!("Provide a super component or an override for this component!");
            };

            Ok(self.delta_for_this(res, gamma)) 
        }

        /// Measure the component by driving the component with the velocity `omega` until either 
        /// the measurement condition is true or the maximum distance `delta` is reached. When the endpoint 
        /// is reached, the controls will set the distance to `set_dist` and return the **relative** distance travelled.
        fn measure(&mut self, mut delta : Delta, mut omega : Omega, mut set_gamma : Gamma) -> Result<Delta, crate::Error> {
            delta = self.delta_for_super(delta, self.gamma());
            omega = self.omega_for_super(omega, self.gamma());
            set_gamma = self.gamma_for_super(set_gamma);

            if let Some(s_comp) = self.super_comp_mut() {
                s_comp.measure(delta, omega, set_gamma)
            } else {
                #[cfg(feature = "std")]
                panic!("Provide a super component or an override for this component!");
            }
        }   
    // 

    // Async
        /// Moves the component by the relative distance as fast as possible
        #[inline(always)]
        #[cfg(feature = "std")]
        fn drive_rel_async(&mut self, mut delta : Delta, mut omega : Omega) -> Result<(), crate::Error> {
            let gamma = self.gamma(); 

            delta = self.delta_for_super(delta, gamma);
            omega = self.omega_for_super(omega, gamma);

            if let Some(s_comp) = self.super_comp_mut() {
                s_comp.drive_rel_async(delta, omega)
            } else {
                #[cfg(feature = "std")]
                panic!("Provide a super component or an override for this component!");
            }
        }

        /// Moves the component to the given position as fast as possible, halts the script until the 
        /// movement is finished and returns the actual **abolute** distance traveled to. 
        #[inline(always)]
        #[cfg(feature = "std")]
        fn drive_abs_async(&mut self, mut gamma : Gamma, mut omega : Omega) -> Result<(), crate::Error> {
            omega = self.omega_for_super(omega, gamma);
            gamma = self.gamma_for_super(gamma);

            if let Some(s_comp) = self.super_comp_mut() {
                s_comp.drive_abs_async(gamma, omega)
            } else {
                #[cfg(feature = "std")]
                panic!("Provide a super component or an override for this component!");
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
        /// - Returns an error if the definition has not been overwritten by the component and no super component is known
        /// - Returns an error if no async movement has been started yet
        #[inline(always)]
        #[cfg(feature = "std")]
        fn await_inactive(&mut self) -> Result<Delta, crate::Error> {
            let delta = if let Some(s_comp) = self.super_comp_mut() {
                s_comp.await_inactive()?
            } else {
                #[cfg(feature = "std")]
                panic!("Provide a super component or an override for this component!");
            }; 

            Ok(self.delta_for_this(delta, self.gamma_for_super(self.gamma())))
        }
    // 

    // Position
        /// Returns the **absolute** position of the component.
        /// 
        /// ```rust
        /// use stepper_lib::{SyncComp, StepperCtrl, StepperConst};
        /// use stepper_lib::comp::Cylinder;
        /// use stepper_lib::units::*;
        /// 
        /// // Position of components
        /// const POS : Gamma = Gamma(10.0);
        /// 
        /// // Create a new cylinder (implements SyncComp)
        /// let mut cylinder = Cylinder::new(
        ///     // Stepper Motor as subcomponent (also implements SyncComp)
        ///     StepperCtrl::new_sim(StepperConst::GEN), 
        /// 0.5);    // Ratio is set to 0.5, which means for each radian the motor moves, the cylinder moves for 0.5 mm
        /// 
        /// cylinder.write_gamma(POS);
        /// 
        /// assert!((cylinder.gamma() - POS).abs() < Delta(0.05));      // Check with small tolerance
        /// ```
        #[inline(always)]
        fn gamma(&self) -> Gamma {
            let super_len = if let Some(s_comp) = self.super_comp() {
                s_comp.gamma()
            } else {
                #[cfg(feature = "std")]
                panic!("Provide a super component or an override for this component!");
            }; 

            self.gamma_for_this(super_len)
        }

        /// Overwrite the current **absolute** position of the component without triggering actual movements. 
        /// 
        /// Be aware that only full steps can be written in distance, meaning that for position comparision a 
        /// small tolerance has to be considered, as the value written won't be the exact gamma value given.
        /// 
        /// ```rust
        /// use stepper_lib::{SyncComp, StepperCtrl, StepperConst};
        /// use stepper_lib::comp::Cylinder;
        /// use stepper_lib::units::*;
        /// 
        /// // Position of components
        /// const POS : Gamma = Gamma(10.0);
        /// 
        /// // Create a new cylinder (implements SyncComp)
        /// let mut cylinder = Cylinder::new(
        ///     // Stepper Motor as subcomponent (also implements SyncComp)
        ///     StepperCtrl::new_sim(StepperConst::GEN), 
        /// 0.5);    // Ratio is set to 0.5, which means for each radian the motor moves, the cylinder moves for 0.5 mm
        /// 
        /// cylinder.write_gamma(POS);
        /// 
        /// assert!((cylinder.gamma() - POS).abs() < Delta(0.05));      // Check with small tolerance
        /// ```
        #[inline(always)]
        fn write_gamma(&mut self, mut gamma : Gamma) {
            gamma = self.gamma_for_super(gamma);

            if let Some(s_comp) = self.super_comp_mut() {
                s_comp.write_gamma(gamma);
            } else {
                #[cfg(feature = "std")]
                panic!("Provide a super component or an override for this component!");
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
        /// use stepper_lib::{SyncComp, StepperCtrl, StepperConst};
        /// use stepper_lib::comp::GearJoint;
        /// use stepper_lib::units::*;
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
        ///     StepperCtrl::new_sim(StepperConst::GEN), 
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
            gamma = self.gamma_for_super(gamma);

            let delta = if let Some(s_comp) = self.super_comp() {
                s_comp.lim_for_gamma(gamma)
            } else {
                #[cfg(feature = "std")]
                panic!("Provide a super component or an override for this component!");
            };

            self.delta_for_this(delta, gamma)
        }

        /// Sets an endpoint in the current direction by modifying the components limits. For example, when the component is moving
        /// in the positive direction and the endpoint is set, this function will overwrite the current maximum limit with the current
        /// gamma value. The component is then not allowed to move in the current direction anymore. 
        /// 
        /// ```rust
        /// use stepper_lib::{SyncComp, StepperCtrl, StepperConst};
        /// use stepper_lib::comp::GearJoint;
        /// use stepper_lib::data::LinkedData;
        /// use stepper_lib::units::*;
        /// 
        /// const GAMMA : Gamma = Gamma(1.0); 
        /// 
        /// // Create a new gear bearing (implements SyncComp)
        /// let mut gear = GearJoint::new(
        ///     // Stepper Motor as subcomponent (also implements SyncComp)
        ///     StepperCtrl::new_sim(StepperConst::GEN), 
        /// 0.5);    // Ratio is set to 0.5, which means for each radian the motor moves, the bearing moves for half a radian
        /// gear.write_link(LinkedData::GEN);           // Link component for driving
        /// 
        /// gear.drive_rel(Delta(-0.1), Omega(50.0));    // Drive component in negative direction
        /// 
        /// gear.set_end(GAMMA);
        /// 
        /// assert_eq!(gear.lim_for_gamma(Gamma(2.0)), Delta::ZERO);     
        /// assert_eq!(gear.lim_for_gamma(Gamma(-2.0)), Delta(-3.0));      
        /// ```
        #[inline(always)]
        fn set_end(&mut self, mut set_gamma : Gamma) {
            set_gamma = self.gamma_for_super(set_gamma);

            if let Some(s_comp) = self.super_comp_mut() {
                s_comp.set_end(set_gamma)
            } else {
                #[cfg(feature = "std")]
                panic!("Provide a super component or an override for this component!");
            }
        }

        /// Set the limits for the minimum and maximum angles that the component can reach, note that the limit will 
        /// be converted and transfered to the super component if defined. 
        /// 
        /// Unlike [SyncComp::reset_limit()], this function does not overwrite the current `min` or `max` limits if they
        /// are set to `None`. 
        /// 
        /// ```rust
        /// use stepper_lib::{SyncComp, StepperCtrl, StepperConst};
        /// use stepper_lib::comp::GearJoint;
        /// use stepper_lib::units::*;
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
        ///     StepperCtrl::new_sim(StepperConst::GEN), 
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
            min = match min {   // Update the value with super component gammas
                Some(min) => Some(self.gamma_for_super(min)),
                None => None
            }; 

            max = match max {   // Update the value with super component gammas
                Some(max) => Some(self.gamma_for_super(max)),
                None => None
            };

            if let Some(s_comp) = self.super_comp_mut() {
                s_comp.set_limit(min, max)      // If the super component exists
            } else {
                #[cfg(feature = "std")]
                panic!("Provide a super component or an override for this component!");
            }
        }

        /// Set the limits for the minimum and maximum angles that the component can reach, note that the limit will 
        /// be converted and transfered to the super component if this component has one. 
        /// 
        /// The difference to [SyncComp::set_limit()] is that this function **overwrites** the current limits set.
        /// 
        /// ```rust
        /// use stepper_lib::{SyncComp, StepperCtrl, StepperConst};
        /// use stepper_lib::comp::GearJoint;
        /// use stepper_lib::units::*;
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
        ///     StepperCtrl::new_sim(StepperConst::GEN), 
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
                Some(min) => Some(self.gamma_for_super(min)),
                None => None
            }; 

            max = match max {
                Some(max) => Some(self.gamma_for_super(max)),
                None => None
            };

            if let Some(s_comp) = self.super_comp_mut() {
                s_comp.reset_limit(min, max)
            } else {
                #[cfg(feature = "std")]
                panic!("Provide a super component or an override for this component!");
            }
        }
    // 

    // Load calculation
        /// Apply a load force to the component, slowing down movements 
        /// 
        /// ```rust
        /// use stepper_lib::{SyncComp, StepperCtrl, StepperConst};
        /// use stepper_lib::comp::GearJoint;
        /// use stepper_lib::units::*;
        /// 
        /// // Force to act upon the component
        /// const FORCE : Force = Force(0.2);
        /// 
        /// // Create a new gear bearing (implements SyncComp)
        /// let mut gear = GearJoint::new(
        ///     // Stepper Motor as subcomponent (also implements SyncComp)
        ///     StepperCtrl::new_sim(StepperConst::GEN), 
        /// 0.5);    // Ratio is set to 0.5, which means for each radian the motor moves, the bearing moves for half a radian
        /// 
        /// gear.apply_force(FORCE);
        /// 
        /// assert_eq!(Gamma(2.0), gear.gamma_for_super(Gamma(1.0)));
        /// assert_eq!(Force(0.1), gear.super_comp().unwrap().vars().t_load);
        /// ```
        #[inline(always)]
        fn apply_force(&mut self, mut force : Force) { // TODO: Add overload protection
            force = Force(self.gamma_for_this(Gamma(force.0)).0);

            if let Some(s_comp) = self.super_comp_mut() {
                s_comp.apply_force(force);
            } else {
                #[cfg(feature = "std")]
                panic!("Provide a super component or an override for this component!");
            }
        }
        
        /// Apply a load inertia to the component, slowing down movements
        /// 
        /// # Panics
        /// 
        /// Panics if no super component or override of the function has been provided.
        /// 
        /// ```rust
        /// use stepper_lib::{SyncComp, StepperCtrl, StepperConst};
        /// use stepper_lib::comp::GearJoint;
        /// use stepper_lib::units::*;
        /// 
        /// // Inertia to act upon the component
        /// const INERTIA : Inertia = Inertia(4.0);
        /// 
        /// // Create a new gear bearing (implements SyncComp)
        /// let mut gear = GearJoint::new(
        ///     // Stepper Motor as subcomponent (also implements SyncComp)
        ///     StepperCtrl::new_sim(StepperConst::GEN), 
        /// 0.5);    // Ratio is set to 0.5, which means for each radian the motor moves, the bearing moves for half a radian
        /// 
        /// // Applies the inertia to the gearbearing component
        /// gear.apply_inertia(INERTIA);
        /// 
        /// assert_eq!(Gamma(2.0), gear.gamma_for_super(Gamma(1.0)));
        /// assert_eq!(Inertia(1.0), gear.super_comp().unwrap().vars().j_load);
        /// ```
        #[inline(always)]
        fn apply_inertia(&mut self, mut inertia : Inertia) {
            inertia = Inertia(self.gamma_for_this(self.gamma_for_this(Gamma(inertia.0))).0);

            if let Some(s_comp) = self.super_comp_mut() {
                s_comp.apply_inertia(inertia);
            } else {
                #[cfg(feature = "std")]
                panic!("Provide a super component or an override for this component!");
            }
        }

        /// Applies a bend factor to the given component, bending down its movements. 
        /// 
        /// # Panics
        /// 
        /// The function panics if the factor given is bigger than 1.0, equal to 0.0 or negative. Also it panics if neither the function 
        /// definition has not been overwritten nor has a super component been provied.
        #[inline(always)]
        fn apply_bend_f(&mut self, f_bend : f32) {
            if let Some(s_comp) = self.super_comp_mut() {
                s_comp.apply_bend_f(f_bend);
            } else {
                #[cfg(feature = "std")]
                panic!("Provide a super component or an override for this component!");
            }
        }
    // 
}

impl dyn SyncComp 
{
    /// Returns the type name of the component as [String]. Used for configuration file parsing.
    #[inline(always)]
    pub fn get_type_name(&self) -> &str {
        type_name::<Self>()
    }
}