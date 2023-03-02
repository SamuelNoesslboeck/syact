extern crate alloc;
use alloc::sync::Arc;

use core::any::type_name;

use crate::{MathActor, Delta, Gamma, Omega, Force, Inertia, Alpha, StepperConst};
use crate::ctrl::SimpleMeas;

// Submodules
/// Module for asynchronous DC-Motors
pub mod asynchr;

mod cylinder;
pub use cylinder::Cylinder;

mod cylinder_triangle;
pub use cylinder_triangle::CylinderTriangle;

mod gear_bearing;
pub use gear_bearing::GearBearing;

mod group;
pub use group::*;

mod lk;
pub use lk::LinkedData;

pub mod tool;
pub use tool::Tool;
//

/// Trait for defining controls and components of synchronous actuators
/// 
/// # Super components
/// 
/// Components can have multiple layers, for example take a stepper motor with a geaerbox attached to it. The stepper motor and both combined will be a component, the later having 
/// the stepper motor component defined as it's super component. (See [GearBearing])
pub trait Component : SimpleMeas + MathActor + core::fmt::Debug
{
    // Data
        /// Returns a copy of the stepper constants used by the [stepper driver](crate::ctrl::StepperDriver)
        fn consts(&self) -> StepperConst;
    // 

    // Super
        /// Returns a readonly reference to the super [Component] if it exists, returns `None` otherwise. If not overwritten by the 
        /// trait implementation, this function returns always `None`.
        /// 
        /// # Example
        /// 
        /// A super component would for example be the stepper motor for a cylinder (See [Cylinder])
        /// 
        /// ```rust
        /// use stepper_lib::{Component, StepperCtrl, StepperConst, Gamma};
        /// use stepper_lib::comp::Cylinder;
        /// 
        /// // Create a new cylinder (implements Component)
        /// let mut cylinder = Cylinder::new(
        ///     // Stepper Motor as subcomponent (also implements Component)
        ///     StepperCtrl::new_sim(StepperConst::GEN), 
        /// 1.0);     
        /// 
        /// // Overwrite the cylinder position
        /// cylinder.write_gamma(Gamma(1.0));
        /// 
        /// // Checks position of super component (The position of the super component won't be exactly 1.0, as the stepper motor can only move whole steps)
        /// assert!((cylinder.super_comp().unwrap().get_gamma() - Gamma(1.0)).abs().0 <= StepperConst::GEN.step_ang());
        /// ```
        #[inline(always)]
        fn super_comp(&self) -> Option<&dyn Component> {
            None
        }

        /// Returns a mutable reference to the super [Component] if it exists, returns `None` otherwise
        /// 
        /// # Example
        /// 
        /// A super component would for example be the stepper motor for a cylinder (See [Cylinder])
        /// 
        /// ```rust
        /// use stepper_lib::{Component, StepperCtrl, StepperConst, Gamma};
        /// use stepper_lib::comp::Cylinder;
        /// 
        /// // Create a new cylinder (implements Component)
        /// let mut cylinder = Cylinder::new(
        ///     // Stepper Motor as subcomponent (also implements Component)
        ///     StepperCtrl::new_sim(StepperConst::GEN), 
        /// 1.0);    
        /// 
        /// // Overwrite position in super component
        /// cylinder.super_comp_mut().unwrap().write_gamma(Gamma(1.0));
        /// 
        /// // Check own position (The position of the component won't be exactly 1.0, as the stepper motor can only move whole steps)
        /// assert!((cylinder.super_comp().unwrap().get_gamma() - Gamma(1.0)).abs().0 <= StepperConst::GEN.step_ang());
        /// ```
        #[inline(always)]
        fn super_comp_mut(&mut self) -> Option<&mut dyn Component> {
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
        /// use stepper_lib::{Component, StepperCtrl, StepperConst, Gamma};
        /// use stepper_lib::comp::Cylinder;
        /// 
        /// // Create a new cylinder (implements Component)
        /// let mut cylinder = Cylinder::new(
        ///     // Stepper Motor as subcomponent (also implements Component)
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
        /// use stepper_lib::{Component, StepperCtrl, StepperConst, Gamma};
        /// use stepper_lib::comp::Cylinder;
        /// 
        /// // Create a new cylinder (implements Component)
        /// let mut cylinder = Cylinder::new(
        ///     // Stepper Motor as subcomponent (also implements Component)
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
        /// use stepper_lib::{Component, StepperCtrl, StepperConst, Gamma, Delta};
        /// use stepper_lib::comp::Cylinder;
        /// 
        /// // Position of components
        /// const POS : Gamma = Gamma(10.0);
        /// 
        /// // Create a new cylinder (implements Component)
        /// let mut cylinder = Cylinder::new(
        ///     // Stepper Motor as subcomponent (also implements Component)
        ///     StepperCtrl::new_sim(StepperConst::GEN), 
        /// 0.5);    // Ratio is set to 2.0, which means for each radian the motor moves, the cylinder moves for 2.0 mm
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

    // Link
        /// Links this component to other components in the group, sharing data that is often relevant for multiple components in a group such as *voltage*
        /// (See [LinkedData])
        #[inline]
        fn link(&mut self, lk : Arc<LinkedData>) {
            if let Some(s_comp) = self.super_comp_mut() {
                s_comp.link(lk);
            }
        }

        /// Returns the [LinkedData] for the component
        // fn get_link(&self) -> &Arc<LinkedData> {
        //     if let Some(s_comp) = self.super_comp() {
        //         s_comp.get_link()
        //     } 
        // }
    // 

    // JSON I/O 
        /// Get the *JSON* data of the current component as [serde_json::Value]
        fn to_json(&self) -> Result<serde_json::Value, serde_json::Error>;
    // 

    /// Moves the component by the relative distance as fast as possible, halts the script until the movement is finshed and returns the actual **absolute** distance traveled
    fn drive_rel(&mut self, mut delta : Delta, mut vel : Omega) -> Delta {
        let gamma = self.get_gamma(); 

        delta = self.delta_for_super(delta, gamma);
        vel = self.omega_for_super(vel, gamma);

        let res = if let Some(s_comp) = self.super_comp_mut() {
            s_comp.drive_rel(delta, vel)
        } else { Delta::ZERO }; 
        
        self.delta_for_this(res, self.gamma_for_super(gamma))
    }

    /// Moves the component by the relative distance as fast as possible. \
    /// To wait unti the movement operation is completed, use the [await_inactive](Component::await_inactive()) function
    #[cfg(feature = "simple_async")]
    fn drive_rel_async(&mut self, mut delta : Delta, mut vel : Omega) {
        delta = self.delta_for_super(delta, self.get_gamma());
        vel = self.omega_for_super(vel, self.get_gamma());

        if let Some(s_comp) = self.super_comp_mut() {
            s_comp.drive_rel_async(delta, vel);
        }
    }

    /// Moves the component to the given position as fast as possible, halts the script until the movement is finished and returns the actual **abolute** distance traveled to. 
    fn drive_abs(&mut self, mut gamma : Gamma, mut omega : Omega) -> Delta {
        omega = self.omega_for_super(omega, gamma);
        gamma = self.gamma_for_super(gamma);

        let res = if let Some(s_comp) = self.super_comp_mut() {
            s_comp.drive_abs(gamma, omega)
        } else { Delta::ZERO }; 

        self.delta_for_this(res, gamma)
    }

    /// Moves the component to the given position as fast as possible. \
    /// To wait unti the movement operation is completed, use the [await_inactive](Component::await_inactive()) function
    #[cfg(feature = "simple_async")]
    fn drive_abs_async(&mut self, mut gamma : Gamma, mut omega : Omega) {
        gamma = self.gamma_for_super(gamma);
        omega = self.omega_for_super(omega, self.get_gamma());

        if let Some(s_comp) = self.super_comp_mut() {
            s_comp.drive_abs_async(gamma, omega);
        }
    }

    /// Measure the component by driving the component with the velocity `omega` until either the measurement condition is true or the maximum distance `delta` 
    /// is reached. When the endpoint is reached, the controls will set the distance to `set_dist`. The lower the `accuracy`, the higher 
    /// are the computational difficulties, as the function checks more often if the measure pin has a HIGH signal
    /// 
    /// # Sync
    /// 
    /// The thread is halted until the measurement is finished
    fn measure(&mut self, mut delta : Delta, mut omega : Omega, mut set_gamma : Gamma, accuracy : u64) -> bool {
        delta = self.delta_for_super(delta, self.get_gamma());
        omega = self.omega_for_super(omega, self.get_gamma());
        set_gamma = self.gamma_for_super(set_gamma);

        if let Some(s_comp) = self.super_comp_mut() {
            s_comp.measure(delta, omega, set_gamma, accuracy)
        } else { false }
    }   

    /// Measure the component by driving the component with the velocity `omega` until either the measurement condition is true or the maximum distance `delta` 
    /// is reached. The lower the `accuracy`, the higher are the computational difficulties, as the function checks more often if the measure pin has a HIGH signal
    #[cfg(feature = "simple_async")]
    fn measure_async(&mut self, mut delta : Delta, mut omega : Omega, accuracy : u64) {
        delta = self.delta_for_super(delta, self.get_gamma());
        omega = self.omega_for_super(omega, self.get_gamma());

        if let Some(s_comp) = self.super_comp_mut() {
            s_comp.measure_async(delta, omega, accuracy)
        } 
    }

    /// Halts the thread until the movement of the component has finished. \
    /// Do only use it after an async movement has been triggered before!
    #[cfg(feature = "simple_async")] 
    fn await_inactive(&self) {
        if let Some(s_comp) = self.super_comp() {
            s_comp.await_inactive();
        } 
    }

    // Position
        /// Returns the **absolute** position of the component
        /// 
        /// # Units
        ///
        /// - Returns either radians or millimeter
        fn get_gamma(&self) -> Gamma {
            let super_len = if let Some(s_comp) = self.super_comp() {
                s_comp.get_gamma()
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

        /// Returns if any limit positions have been reached. The value returned can either be radians or millimeters, depending on the type of component
        /// 
        /// # Limits
        /// 
        /// If the return value
        /// - greater than 0, the maximum has been reached by the returned amount
        /// - is smaller than 0, the minimum has been reached by the returned amount
        /// - equal to 0, no limit has been reached
        /// - NaN, no limit has been set yet
        fn get_limit_dest(&self, mut gamma : Gamma) -> Delta {
            gamma = self.gamma_for_super(gamma);

            let delta = if let Some(s_comp) = self.super_comp() {
                s_comp.get_limit_dest(gamma)
            } else { Delta::ZERO };

            self.delta_for_this(delta, gamma)
        }

        fn set_endpoint(&mut self, mut set_gamma : Gamma) -> bool {
            set_gamma = self.gamma_for_super(set_gamma);

            if let Some(s_comp) = self.super_comp_mut() {
                s_comp.set_endpoint(set_gamma)
            } else { false }
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
        fn apply_load_force(&mut self, mut force : Force) { // TODO: Add overload protection
            force = Force(self.gamma_for_this(Gamma(force.0)).0);

            if let Some(s_comp) = self.super_comp_mut() {
                s_comp.apply_load_force(force);
            }
        }
        
        /// Apply a load inertia to the component, slowing down movements
        fn apply_load_inertia(&mut self, mut inertia : Inertia) {
            inertia = Inertia(self.gamma_for_this(self.gamma_for_this(Gamma(inertia.0))).0);

            if let Some(s_comp) = self.super_comp_mut() {
                s_comp.apply_load_inertia(inertia);
            }
        }
    // 
}

impl dyn Component 
{
    /// Returns the type name of the component as [String]. Used for configuration file parsing
    #[inline(always)]
    pub fn get_type_name(&self) -> &str {
        type_name::<Self>()
    }
}