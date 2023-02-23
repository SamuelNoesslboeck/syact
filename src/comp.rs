extern crate alloc;
use alloc::sync::Arc;

use core::any::type_name;

use crate::MathActor;
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

mod tool;
pub use tool::*;
//

/// Trait for defining controls and components of synchronous actuators
/// 
/// # Super components
/// Components can have multiple layers, for example take a stepper motor with a geaerbox attached to it. The stepper motor and both combined will be a component, the later having 
/// the stepper motor component defined as it's super component. (See [GearBearing])
pub trait Component : SimpleMeas + MathActor + core::fmt::Debug
{
    // Super
        /// Returns a readonly reference to the super [Component](self) if it exists, returns `None` otherwise
        /// 
        /// # Example
        /// 
        /// A super component would for example be the stepper motor for a cylinder (See [Cylinder])
        #[inline(always)]
        fn super_comp(&self) -> Option<&dyn Component> {
            None
        }

        /// Returns a mutable reference to the super [Component](self) if it exists, returns `None` otherwise
        /// 
        /// # Example
        /// 
        /// A super component would for example be the stepper motor for a cylinder (See [Cylinder])
        #[inline(always)]
        fn super_comp_mut(&mut self) -> Option<&mut dyn Component> {
            None
        }

        /// Converts the given **absolute** distance to the **absolute** distance for the super component
        /// 
        /// # Example
        /// 
        /// When using a gearmotor with a ratio of four (motor movement speed will be reduced to a quater), 
        /// this function will return a super distance *four times higher* than the input distance
        #[inline(always)]
        fn dist_for_super(&self, this_len : f32) -> f32 {
            this_len
        }

        /// Converts the given **absolute** distance for the super component to the **absolute** distance for this component
        /// 
        /// # Example
        /// 
        /// When using a gearmotor with a ratio of four (motor movement speed will be reduced to a quater), 
        /// this function will return a distance *four times higher* than the input super distance
        #[inline(always)]
        fn dist_for_this(&self, super_len : f32) -> f32 {
            super_len
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

    /// Moves the component by the relative distance as fast as possible, halts the script until the movement is finshed and returns the actual distance traveled
    /// 
    /// # Units
    /// 
    ///  - The distance `dist` can be either an angle (Unit radians) or a distancce (Unit mm)
    ///  - The velocity `vel` is the maximum change rate of the distance, either angular velocity (Unit radians per secoond) or linear velocity (Unit mm per second)
    fn drive_rel(&mut self, mut dist : f32, mut vel : f32) -> f32 {
        dist = self.dist_for_super(dist);
        vel = self.dist_for_super(vel);

        let res = if let Some(s_comp) = self.super_comp_mut() {
            s_comp.drive_rel(dist, vel)
        } else { 0.0 }; 
        
        self.dist_for_this(res)
    }

    /// Moves the component by the relative distance as fast as possible
    /// 
    /// # Units
    /// 
    ///  - The distance `dist` can be either an angle (Unit radians) or a distancce (Unit mm)
    ///  - The velocity `vel` is the maximum change rate of the distance, either angular velocity (Unit radians per secoond) or linear velocity (Unit mm per second)
    /// To wait unti the movement operation is completed, use the [await_inactive](Component::await_inactive()) function
    fn drive_rel_async(&mut self, mut dist : f32, mut vel : f32) {
        dist = self.dist_for_super(dist);
        vel = self.dist_for_super(vel);

        if let Some(s_comp) = self.super_comp_mut() {
            s_comp.drive_rel_async(dist, vel);
        }
    }

    /// Moves the component to the given position as fast as possible, halts the script until the movement is finished and returns the actual distance traveled
    /// 
    /// # Units
    /// 
    ///  - The distance `dist` can be either an angle (Unit radians) or a distancce (Unit mm), the value should represent the **absolute position**
    ///  - The velocity `vel` is the maximum change rate of the distance, either angular velocity (Unit radians per secoond) or linear velocity (Unit mm per second)
    fn drive_abs(&mut self, mut dist : f32, mut vel : f32) -> f32 {
        dist = self.dist_for_super(dist);
        vel = self.dist_for_super(vel);

        let res = if let Some(s_comp) = self.super_comp_mut() {
            s_comp.drive_abs(dist, vel)
        } else { 0.0 }; 

        self.dist_for_this(res)
    }

    /// Moves the component to the given position as fast as possible
    /// 
    /// # Units
    /// 
    ///  - The distance `dist` can be either an angle (Unit radians) or a distancce (Unit mm), the value should represent the **absolute position**
    ///  - The velocity `vel` is the maximum change rate of the distance, either angular velocity (Unit radians per secoond) or linear velocity (Unit mm per second)
    /// To wait unti the movement operation is completed, use the [await_inactive](Component::await_inactive()) function
    fn drive_abs_async(&mut self, mut dist : f32, mut vel : f32) {
        dist = self.dist_for_super(dist);
        vel = self.dist_for_super (vel);

        if let Some(s_comp) = self.super_comp_mut() {
            s_comp.drive_abs_async(dist, vel);
        }
    }

    /// Measure the component by driving the component with the velocity `vel` until either the measurement condition is true or the maximum distance `dist` 
    /// is reached. When the endpoint is reached, the controls will set the distance to `set_dist`. The lower the `accuracy`, the higher 
    /// are the computational difficulties, as the function checks more often if the measure pin has a HIGH signal
    /// 
    /// ### Sync
    /// 
    /// The thread is halted until the measurement is finished
    fn measure(&mut self, mut dist : f32, mut vel : f32, mut set_dist : f32, accuracy : u64) -> bool {
        dist = self.dist_for_super(dist);
        vel = self.dist_for_super(vel);
        set_dist = self.dist_for_super(set_dist);

        if let Some(s_comp) = self.super_comp_mut() {
            s_comp.measure(dist, vel, set_dist, accuracy)
        } else { false }
    }   

    /// Measure the component by driving the component with the velocity `vel` until either the measurement condition is true or the maximum distance `dist` 
    /// is reached. The lower the `accuracy`, the higher are the computational difficulties, as the function checks more often if the measure pin has a HIGH signal
    #[cfg(feature = "simple_async")]
    fn measure_async(&mut self, mut dist : f32, mut vel : f32, accuracy : u64) {
        dist = self.dist_for_super(dist);
        vel = self.dist_for_super(vel);

        if let Some(s_comp) = self.super_comp_mut() {
            s_comp.measure_async(dist, vel, accuracy)
        } 
    }

    /// Halts the thread until the movement of the component has finished. \
    /// Do only use it after an async movement has been triggered before!
    #[cfg(feature = "std")] 
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
        fn get_dist(&self) -> f32 {
            let super_len = if let Some(s_comp) = self.super_comp() {
                s_comp.get_dist()
            } else { 0.0 };

            self.dist_for_this(super_len)
        }

        /// Overwrite the current **absolute** position of the component without triggering actual movements
        /// 
        /// # Units
        /// 
        ///  - `dist` Either radians or millimeters
        fn write_dist(&mut self, mut dist : f32) {
            dist = self.dist_for_super(dist);

            if let Some(s_comp) = self.super_comp_mut() {
                s_comp.write_dist(dist);
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
        fn get_limit_dest(&self, mut dist : f32) -> f32 {
            dist = self.dist_for_super(dist);

            if let Some(s_comp) = self.super_comp() {
                s_comp.get_limit_dest(dist)
            } else { 0.0 }
        }

        ///
        fn set_endpoint(&mut self, mut set_dist : f32) -> bool {
            set_dist = self.dist_for_super(set_dist);

            if let Some(s_comp) = self.super_comp_mut() {
                s_comp.set_endpoint(set_dist)
            } else { false }
        }

        ///
        fn set_limit(&mut self, mut limit_min : Option<f32>, mut limit_max : Option<f32>) {
            limit_min = match limit_min {
                Some(min) => Some(self.dist_for_super(min)),
                None => None
            }; 

            limit_max = match limit_max {
                Some(max) => Some(self.dist_for_super(max)),
                None => None
            };

            if let Some(s_comp) = self.super_comp_mut() {
                s_comp.set_limit(limit_min, limit_max)
            }
        }
    // 

    // Load calculation
        /// Apply a load force to the component, slowing down movements 
        /// 
        /// # Units
        /// 
        /// The unit of the `force` can either be *Newton* or *Newtonmeters*, depending on the component
        fn apply_load_force(&mut self, mut force : f32) { // TODO: Add overload protection
            force = self.dist_for_this(force);

            if let Some(s_comp) = self.super_comp_mut() {
                s_comp.apply_load_force(force);
            }
        }
        
        /// Apply a load inertia to the component, slowing down movements
        /// 
        /// # Units
        /// 
        /// The unit of the `inertia` can either be *kilogramm* or *kilogramm-meter^2*
        fn apply_load_inertia(&mut self, mut inertia : f32) {
            inertia = self.dist_for_this(self.dist_for_this(inertia));

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