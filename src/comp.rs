use std::any::type_name;
use std::sync::Arc;

use crate::{SimpleMeas, MathActor};

// Submodules
pub mod asynchr;

mod cylinder;
pub use cylinder::*;

mod cylinder_triangle;
pub use cylinder_triangle::*;

mod gear_bearing;
pub use gear_bearing::*;

mod group;
pub use group::*;

mod lk;
pub use lk::*;

mod tool;
pub use tool::*;
//

/// Trait for defining controls and components
pub trait Component : SimpleMeas + MathActor + std::fmt::Debug
{
    // Super
        #[inline]
        fn super_comp(&self) -> Option<&dyn Component> {
            None
        }

        #[inline]
        fn super_comp_mut(&mut self) -> Option<&mut dyn Component> {
            None
        }

        #[inline]
        fn dist_for_super(&self, this_len : f32) -> f32 {
            this_len
        }

        #[inline]
        fn dist_for_this(&self, super_len : f32) -> f32 {
            super_len
        }
    // 

    // Link
        fn link(&mut self, lk : Arc<LinkedData>) {
            if let Some(s_comp) = self.super_comp_mut() {
                s_comp.link(lk);
            }
        }

        // fn get_link(&self) -> Arc<LinkedData>;
    // 

    // JSON I/O 
        fn get_type_name(&self) -> String {
            String::from(type_name::<Self>())
        }

        fn to_json(&self) -> serde_json::Value;
    // 

    /// Move the component to the given position as fast as possible and returns the actual distance traveled
    ///  - The distance `dist` can be either an angle (Unit radians) or a distancce (Unit mm)
    ///  - The velocity `vel` is the maximum change rate of the distance, either angular velocity (Unit radians per secoond) or linear velocity (mm per second)
    fn drive_rel(&mut self, mut dist : f32, mut vel : f32) -> f32 {
        dist = self.dist_for_super(dist);
        vel = self.dist_for_super(vel);

        let res = if let Some(s_comp) = self.super_comp_mut() {
            s_comp.drive_rel(dist, vel)
        } else { 0.0 }; 

        self.dist_for_this(res)
    }

    /// Move the component to the given position as fast as possible
    ///  - The distance `dist` can be either an angle (Unit radians) or a distancce (Unit mm)
    ///  - The velocity `vel` is the maximum change rate of the distance, either angular velocity (Unit radians per secoond) or linear velocity (mm per second) \
    /// To wait unti the movement operation is completed, use the `await inactive` function
    fn drive_rel_async(&mut self, mut dist : f32, mut vel : f32) {
        dist = self.dist_for_super(dist);
        vel = self.dist_for_super(vel);

        if let Some(s_comp) = self.super_comp_mut() {
            s_comp.drive_rel_async(dist, vel);
        }
    }

    fn drive_abs(&mut self, mut dist : f32, mut vel : f32) -> f32 {
        dist = self.dist_for_super(dist);
        vel = self.dist_for_super(vel);

        let res = if let Some(s_comp) = self.super_comp_mut() {
            s_comp.drive_abs(dist, vel)
        } else { 0.0 }; 

        self.dist_for_this(res)
    }

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
    fn measure(&mut self, mut dist : f32, mut vel : f32, mut set_dist : f32, accuracy : u64) -> bool {
        dist = self.dist_for_super(dist);
        vel = self.dist_for_super(vel);
        set_dist = self.dist_for_super(set_dist);

        if let Some(s_comp) = self.super_comp_mut() {
            s_comp.measure(dist, vel, set_dist, accuracy)
        } else { false }
    }

    fn measure_async(&mut self, mut dist : f32, mut vel : f32, accuracy : u64) {
        dist = self.dist_for_super(dist);
        vel = self.dist_for_super(vel);

        if let Some(s_comp) = self.super_comp_mut() {
            s_comp.measure_async(dist, vel, accuracy)
        } 
    }

    fn await_inactive(&self) {
        if let Some(s_comp) = self.super_comp() {
            s_comp.await_inactive();
        } 
    }

    // Position
        fn get_dist(&self) -> f32 {
            let super_len = if let Some(s_comp) = self.super_comp() {
                s_comp.get_dist()
            } else { 0.0 };

            self.dist_for_this(super_len)
        }

        fn write_dist(&mut self, mut dist : f32) {
            dist = self.dist_for_super(dist);

            if let Some(s_comp) = self.super_comp_mut() {
                s_comp.write_dist(dist);
            }
        }

        fn get_limit_dest(&self, mut dist : f32) -> f32 {
            dist = self.dist_for_super(dist);

            if let Some(s_comp) = self.super_comp() {
                s_comp.get_limit_dest(dist)
            } else { 0.0 }
        }

        fn set_endpoint(&mut self, mut set_dist : f32) -> bool {
            set_dist = self.dist_for_super(set_dist);

            if let Some(s_comp) = self.super_comp_mut() {
                s_comp.set_endpoint(set_dist)
            } else { false }
        }

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
        fn apply_load_force(&mut self, mut force : f32) {
            force = self.dist_for_this(force);

            if let Some(s_comp) = self.super_comp_mut() {
                s_comp.apply_load_force(force);
            }
        }

        fn apply_load_inertia(&mut self, mut inertia : f32) {
            inertia = self.dist_for_this(self.dist_for_this(inertia));

            if let Some(s_comp) = self.super_comp_mut() {
                s_comp.apply_load_inertia(inertia);
            }
        }
    // 
}