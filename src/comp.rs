use std::{any::type_name, sync::Arc, fmt::Debug};

use serde::{Serialize, Deserialize};

use super::*;
use crate::ctrl::LimitDest;

// Submodules
mod cylinder;
mod cylinder_triangle;
mod gear_bearing;
mod tool;

pub use cylinder::*;
pub use cylinder_triangle::*;
pub use gear_bearing::*;
pub use tool::*;
//

#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct LinkedData 
{
    pub u : f32,
    pub s_f : f32
}

impl LinkedData 
{
    pub const EMPTY : Self = Self {
        u : 0.0, 
        s_f : 0.0
    };
}

impl From<(f32, f32)> for LinkedData 
{
    fn from(data: (f32, f32)) -> Self {
        Self {
            u: data.0, 
            s_f: data.1
        }
    }
}

/// Trait for defining controls and components
pub trait Component : SimpleMeas + MathActor + Debug
{
    // Super
        fn super_comp(&self) -> Option<&dyn Component> {
            None
        }

        fn super_comp_mut(&mut self) -> Option<&mut dyn Component> {
            None
        }

        fn dist_for_super(&self, this_len : f32) -> f32 {
            this_len
        }

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
    fn drive(&mut self, mut dist : f32, mut vel : f32) -> f32 {
        dist = self.dist_for_super(dist);
        vel = self.dist_for_super(vel);

        let res = if let Some(s_comp) = self.super_comp_mut() {
            s_comp.drive(dist, vel)
        } else { 0.0 }; 

        self.dist_for_this(res)
    }

    /// Move the component to the given position as fast as possible
    ///  - The distance `dist` can be either an angle (Unit radians) or a distancce (Unit mm)
    ///  - The velocity `vel` is the maximum change rate of the distance, either angular velocity (Unit radians per secoond) or linear velocity (mm per second) \
    /// To wait unti the movement operation is completed, use the `await inactive` function
    fn drive_async(&mut self, mut dist : f32, mut vel : f32) {
        dist = self.dist_for_super(dist);
        vel = self.dist_for_super(vel);

        if let Some(s_comp) = self.super_comp_mut() {
            s_comp.drive_async(dist, vel);
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

    // fn lin_move(&mut self, dist : f32, vel : f32, vel_max : f32) -> f32;

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

        fn get_limit_dest(&self, pos : f32) -> LimitDest;

        fn set_endpoint(&mut self, mut set_dist : f32) -> bool {
            set_dist = self.dist_for_super(set_dist);

            if let Some(s_comp) = self.super_comp_mut() {
                s_comp.set_endpoint(set_dist)
            } else { false }
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


pub trait ComponentGroup<const N : usize> 
{
    // Data
        fn comps(&self) -> &[Box<dyn Component>; N];

        fn comps_mut(&mut self) -> &mut [Box<dyn Component>; N];

        fn link(&mut self, lk : Arc<LinkedData>) {
            for i in 0 .. N {
                self.comps_mut()[i].link(lk.clone())
            }
        }
    //

    fn drive(&mut self, dist : [f32; N], vel : [f32; N]) -> [f32; N] {
        let mut res = [0.0; N];
        for i in 0 .. N {
            res[i] = self.comps_mut()[i].drive(dist[i], vel[i]);
        }
        res
    }

    fn drive_abs(&mut self, dist : [f32; N], vel : [f32; N]) -> [f32; N] {
        let mut res = [0.0; N];
        for i in 0 .. N {
            res[i] = self.comps_mut()[i].drive(dist[i], vel[i]);
        }
        res
    }

    fn drive_async(&mut self, dist : [f32; N], vel : [f32; N]) {
        for i in 0 .. N {
            self.comps_mut()[i].drive_async(dist[i], vel[i]);
        }
    }

    fn drive_async_abs(&mut self, dist : [f32; N], vel : [f32; N]) {
        for i in 0 .. N {
            self.comps_mut()[i].drive_abs_async(dist[i], vel[i]);
        }
    }

    fn measure(&mut self, dist : [f32; N], vel : [f32; N], set_dist : [f32; N], accuracy : [u64; N]) -> [bool; N] {
        let mut res = [false; N];
        for i in 0 .. N {
            res[i] = self.comps_mut()[i].measure(dist[i], vel[i], set_dist[i], accuracy[i])
        }
        res
    }

    fn measure_async(&mut self, dist : [f32; N], vel : [f32; N], accuracy : [u64; N]) {
        for i in 0 .. N {
            self.comps_mut()[i].measure_async(dist[i], vel[i], accuracy[i])
        }
    }

    fn await_inactive(&self) {
        for i in 0 .. N {
            self.comps()[i].await_inactive();
        }
    }

    // Position
        fn get_dist(&self) -> [f32; N] {
            let mut dists = [0.0; N];
            for i in 0 .. N {
                dists[i] = self.comps()[i].get_dist();
            }
            dists
        }
        
        fn write_dist(&mut self, angles : &[f32; N]) {
            for i in 0 .. N {
                self.comps_mut()[i].write_dist(angles[i])
            }
        }

        fn get_limit_dest(&self, dist : [f32; N]) -> [LimitDest; N] {
            let mut limits = [LimitDest::NotReached; N]; 
            for i in 0 .. N {
                limits[i] = self.comps()[i].get_limit_dest(dist[i]);
            }
            limits
        }

        fn valid_dist(&self, dist : [f32; N]) -> bool {
            let mut res = true;
            for i in 0 .. N {
                res = res & ((!self.comps()[i].get_limit_dest(dist[i]).reached()) & dist[i].is_finite()); 
            }
            res
        }

        fn valid_dist_verb(&self, dist : [f32; N]) -> [bool; N] {
            let mut res = [true; N];
            for i in 0 .. N {
                res[i] = (!self.comps()[i].get_limit_dest(dist[i]).reached()) & dist[i].is_finite(); 
            }
            res
        }

        fn set_endpoint(&mut self, set_dist : [f32; N]) -> [bool; N] {
            let mut res = [false; N];
            for i in 0 .. N {
                res[i] = self.comps_mut()[i].set_endpoint(set_dist[i]);
            }   
            res
        }
    //

    // Load calculation
        fn apply_load_inertia(&mut self, inertias : [f32; N]) {
            for i in 0 .. N {
                self.comps_mut()[i].apply_load_inertia(inertias[i]);
            }
        }

        fn apply_load_force(&mut self, forces : [f32; N]) {
            for i in 0 .. N {
                self.comps_mut()[i].apply_load_force(forces[i]);
            }
        }
    // 
}

impl<const N : usize> ComponentGroup<N> for [Box<dyn Component>; N] 
{
    fn comps(&self) -> &[Box<dyn Component>; N] {
        self
    }
    
    fn comps_mut(&mut self) -> &mut [Box<dyn Component>; N] {
        self
    }
}