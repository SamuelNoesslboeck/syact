use core::any::type_name;

use crate::data::{CompVars, LinkedData};
use crate::units::*;

// Submodules
mod cylinder;
pub use cylinder::Cylinder;

mod cylinder_triangle;
pub use cylinder_triangle::CylinderTriangle;

mod gear_bearing;
pub use gear_bearing::GearBearing;

pub mod group;
pub use group::ComponentGroup;

pub mod tool;
pub use tool::Tool;
//

#[inline(always)]
fn no_super() -> crate::Error {
    crate::Error::new(std::io::ErrorKind::NotFound, "No super component has been found")
}

/// Trait for defining controls and components of synchronous actuators
/// 
/// # Super components
/// 
/// Components can have multiple layers, for example take a stepper motor with a geaerbox attached to it. The stepper motor and both combined will be a component, the later having 
/// the stepper motor component defined as it's super component. (See [GearBearing])
pub trait SyncComp : crate::meas::SimpleMeas + crate::math::MathActor + core::fmt::Debug {
    // Init 
        fn setup(&mut self);

        #[cfg(feature = "std")]
        fn setup_async(&mut self);
    // 

    // Data
        fn vars<'a>(&'a self) -> &'a CompVars;

        fn link<'a>(&'a self) -> &'a LinkedData;

        #[inline]
        fn write_link(&mut self, lk : crate::data::LinkedData) {
            if let Some(s_comp) = self.super_comp_mut() {
                s_comp.write_link(lk);
            }
        }

        // JSON I/O 
        /// Get the *JSON* data of the current component as [serde_json::Value]
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

        #[inline(always)]
        #[allow(unused_variables)]
        fn omega_for_this(&self, super_omega : Omega, this_gamma : Gamma) -> Omega {
            Omega(self.gamma_for_this(Gamma(super_omega.into())).into())
        }

        #[inline(always)]
        #[allow(unused_variables)]
        fn alpha_for_super(&self, this_alpha : Alpha, this_gamma : Gamma) -> Alpha {
            Alpha(self.gamma_for_super(Gamma(this_alpha.into())).into())
        }

        #[inline(always)]
        #[allow(unused_variables)]
        fn alpha_for_this(&self, super_alpha : Alpha, super_gamma : Gamma) -> Alpha {
            Alpha(self.gamma_for_this(Gamma(super_alpha.into())).into())
        }
    // 

    // Movement
        /// Moves the component by the relative distance as fast as possible, halts the script until 
        /// the movement is finshed and returns the actual **absolute** distance traveled
        fn drive_rel(&mut self, mut delta : Delta, mut omega : Omega) -> Result<Delta, crate::Error> {
            let gamma = self.gamma(); 

            delta = self.delta_for_super(delta, gamma);
            omega = self.omega_for_super(omega, gamma);

            let res = if let Some(s_comp) = self.super_comp_mut() {
                s_comp.drive_rel(delta, omega)?
            } else { 
                return Err(no_super()); 
            }; 
            
            Ok(self.delta_for_this(res, self.gamma_for_super(gamma)))
        }

        /// Moves the component to the given position as fast as possible, halts the script until the 
        /// movement is finished and returns the actual **abolute** distance traveled to. 
        fn drive_abs(&mut self, mut gamma : Gamma, mut omega : Omega) -> Result<Delta, crate::Error> {
            omega = self.omega_for_super(omega, gamma);
            gamma = self.gamma_for_super(gamma);

            let res = if let Some(s_comp) = self.super_comp_mut() {
                s_comp.drive_abs(gamma, omega)?
            } else { Delta::ZERO }; 

            Ok(self.delta_for_this(res, gamma)) 
        }

        /// Measure the component by driving the component with the velocity `omega` until either 
        /// the measurement condition is true or the maximum distance `delta` is reached. When the endpoint 
        /// is reached, the controls will set the distance to `set_dist`. The lower the `accuracy`, the higher 
        /// are the computational difficulties, as the function checks more often if the measure pin has a HIGH signal
        /// 
        /// # Sync
        /// 
        /// The thread is halted until the measurement is finished
        fn measure(&mut self, mut delta : Delta, mut omega : Omega, mut set_gamma : Gamma) -> Result<Delta, crate::Error> {
            delta = self.delta_for_super(delta, self.gamma());
            omega = self.omega_for_super(omega, self.gamma());
            set_gamma = self.gamma_for_super(set_gamma);

            if let Some(s_comp) = self.super_comp_mut() {
                s_comp.measure(delta, omega, set_gamma)
            } else { Err(crate::Error::new(std::io::ErrorKind::NotFound, "No super component has been found")) }
        }   
    // 

    // Async
        #[cfg(feature = "std")]
        fn drive_rel_async(&mut self, mut delta : Delta, mut omega : Omega) -> Result<(), crate::Error> {
            let gamma = self.gamma(); 

            delta = self.delta_for_super(delta, gamma);
            omega = self.omega_for_super(omega, gamma);

            if let Some(s_comp) = self.super_comp_mut() {
                s_comp.drive_rel_async(delta, omega)?
            }
            
            Err(no_super())
        }

        /// Moves the component to the given position as fast as possible, halts the script until the 
        /// movement is finished and returns the actual **abolute** distance traveled to. 
        #[cfg(feature = "std")]
        fn drive_abs_async(&mut self, mut gamma : Gamma, mut omega : Omega) -> Result<(), crate::Error> {
            omega = self.omega_for_super(omega, gamma);
            gamma = self.gamma_for_super(gamma);

            if let Some(s_comp) = self.super_comp_mut() {
                s_comp.drive_abs_async(gamma, omega)?
            }

            Err(no_super())
        }

        #[cfg(feature = "std")]
        fn await_inactive(&mut self) -> Result<(), crate::Error> {
            if let Some(s_comp) = self.super_comp_mut() {
                s_comp.await_inactive()? 
            }

            Err(no_super())
        }
    // 

    // Position
        /// Returns the **absolute** position of the component
        /// 
        /// # Units
        ///
        /// - Returns either radians or millimeter
        fn gamma(&self) -> Gamma {
            let super_len = if let Some(s_comp) = self.super_comp() {
                s_comp.gamma()
            } else { Gamma::ZERO };

            self.gamma_for_this(super_len)
        }

        /// Overwrite the current **absolute** position of the component without triggering actual movements
        /// 
        /// # Units
        /// 
        ///  - `dist` Either radians or millimeters
        fn write_gamma(&mut self, mut gamma : Gamma) {
            gamma = self.gamma_for_super(gamma);

            if let Some(s_comp) = self.super_comp_mut() {
                s_comp.write_gamma(gamma);
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
        fn lim_for_gamma(&self, mut gamma : Gamma) -> Delta {
            gamma = self.gamma_for_super(gamma);

            let delta = if let Some(s_comp) = self.super_comp() {
                s_comp.lim_for_gamma(gamma)
            } else { Delta::ZERO };

            self.delta_for_this(delta, gamma)
        }

        fn set_end(&mut self, mut set_gamma : Gamma) {
            set_gamma = self.gamma_for_super(set_gamma);

            if let Some(s_comp) = self.super_comp_mut() {
                s_comp.set_end(set_gamma)
            }
        }

        fn set_limit(&mut self, mut min : Option<Gamma>, mut max : Option<Gamma>) {
            min = match min {
                Some(min) => Some(self.gamma_for_super(min)),
                None => None
            }; 

            max = match max {
                Some(max) => Some(self.gamma_for_super(max)),
                None => None
            };

            if let Some(s_comp) = self.super_comp_mut() {
                s_comp.set_limit(min, max)
            }
        }
    // 

    // Load calculation
        /// Apply a load force to the component, slowing down movements 
        fn apply_force(&mut self, mut force : Force) { // TODO: Add overload protection
            force = Force(self.gamma_for_this(Gamma(force.0)).0);

            if let Some(s_comp) = self.super_comp_mut() {
                s_comp.apply_force(force);
            }
        }
        
        /// Apply a load inertia to the component, slowing down movements
        /// 
        /// ```rust
        /// use stepper_lib::{SyncComp, StepperCtrl, StepperConst};
        /// use stepper_lib::comp::GearBearing;
        /// use stepper_lib::units::*;
        /// 
        /// // Position of components
        /// const INERTIA : Inertia = Inertia(4.0);
        /// 
        /// // Create a new gear bearing (implements SyncComp)
        /// let mut gear = GearBearing::new(
        ///     // Stepper Motor as subcomponent (also implements SyncComp)
        ///     StepperCtrl::new_sim(StepperConst::GEN), 
        /// 0.5);    // Ratio is set to 0.5, which means for each radian the motor moves, the bearing moves for half a radian
        /// 
        /// gear.apply_inertia(INERTIA);
        /// 
        /// assert_eq!(Gamma(2.0), gear.gamma_for_super(Gamma(1.0)));
        /// assert_eq!(Inertia(1.0), gear.super_comp().unwrap().vars().j_load);
        /// ```
        fn apply_inertia(&mut self, mut inertia : Inertia) {
            inertia = Inertia(self.gamma_for_this(self.gamma_for_this(Gamma(inertia.0))).0);

            if let Some(s_comp) = self.super_comp_mut() {
                s_comp.apply_inertia(inertia);
            }
        }
    // 
}

impl dyn SyncComp 
{
    /// Returns the type name of the component as [String]. Used for configuration file parsing
    #[inline(always)]
    pub fn get_type_name(&self) -> &str {
        type_name::<Self>()
    }
}