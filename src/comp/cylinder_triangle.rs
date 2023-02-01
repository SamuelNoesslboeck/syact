use std::sync::Arc;

use serde::{Serialize, Deserialize};

use crate::{Component, LinkedData};
use crate::comp::Cylinder;
use crate::ctrl::{LimitType, LimitDest, SimpleMeas}; 
use crate::math::MathActor;

#[derive(Serialize, Deserialize)]
pub struct CylinderTriangle 
{
    // Cylinder
    pub cylinder : Cylinder,

    // Triangle
    pub l_a : f32,
    pub l_b : f32,

    // Offsets
    pub offset_a : Option<f32>,
    pub offset_b : Option<f32>
}

impl CylinderTriangle 
{
    pub fn new(cylinder : Cylinder, l_a : f32, l_b : f32, offset_a : Option<f32>, offset_b : Option<f32>) -> Self
    {
        let mut tri = CylinderTriangle {
            l_a, 
            l_b,

            cylinder,

            offset_a,
            offset_b,
        };

        tri.cylinder.write_dist(l_a.max(l_b));

        return tri;
    }

    // Conversions
        // Other angles
        pub fn alpha_for_gam(&self, gam : f32) -> f32 {
            (self.l_a / self.dist_for_super(gam) * gam.sin()).asin()
        }

        pub fn beta_for_gam(&self, gam : f32) -> f32 {
            (self.l_b / self.dist_for_super(gam) * gam.sin()).asin()
        }

        pub fn omega_for_gam(&self, vel : f32, gam : f32) -> f32 {
            vel / self.l_a * self.beta_for_gam(gam).sin()
        }

        pub fn vel_for_gam(&self, omega : f32, gam : f32) -> f32 {
            omega * self.l_a / self.beta_for_gam(gam).sin()
        }
    //

    // Limit
        pub fn set_limit(&mut self, limit_min : LimitType, limit_max : LimitType) {
            self.cylinder.set_limit(limit_max, limit_min)
        }
    //
}

impl MathActor for CylinderTriangle
{
    fn accel_dyn(&self, vel : f32, pos : f32) -> f32 {
        self.omega_for_gam(self.cylinder.accel_dyn(self.vel_for_gam(vel, pos), pos), pos)
    }
}

impl SimpleMeas for CylinderTriangle
{
    fn init_meas(&mut self, pin_meas : u16) {
        self.cylinder.init_meas(pin_meas)
    }
}

impl Component for CylinderTriangle {
    // Super 
        fn super_comp(&self) -> Option<&dyn Component> {
            Some(&self.cylinder)
        }

        fn super_comp_mut(&mut self) -> Option<&mut dyn Component> {
            Some(&mut self.cylinder)
        }

        /// Returns the cylinder length for the given angle gamma
        fn dist_for_super(&self, gam : f32) -> f32 {
            (self.l_a.powi(2) + self.l_b.powi(2) - 2.0 * self.l_a * self.l_b * gam.cos()).powf(0.5)
        }

        /// Returns the angle gamma for the given cylinder length _(len < (l_a + l_b))_
        fn dist_for_this(&self, len : f32) -> f32 {
            ((self.l_a.powi(2) + self.l_b.powi(2) - len.powi(2)) / 2.0 / self.l_a / self.l_b).acos()
        }
    // 

    // Link
        fn link(&mut self, lk : Arc<LinkedData>) {
            self.cylinder.link(lk);
        }
    //

    // JSON I/O
        fn to_json(&self) -> serde_json::Value {
            serde_json::to_value(self).unwrap()
        }
    //

    /// See [Component::drive()](`Component::drive()`)
    /// - `dist`is the angular distance to be moved (Unit radians)
    /// - `vel` is the cylinders extend velocity (Unit mm per second)
    fn drive(&mut self, dist : f32, vel : f32) -> f32 {
        self.cylinder.drive(self.dist_for_super(dist + self.get_dist()) - self.cylinder.get_dist(), vel)
    }

    /// See [Component::drive_async()](`Component::drive_async()`)
    /// - `dist`is the angular distance to be moved (Unit radians)
    /// - `vel` is the cylinders extend velocity (Unit mm per second)
    fn drive_async(&mut self, dist : f32, vel : f32) {
        self.cylinder.drive_async(self.dist_for_super(dist + self.get_dist()) - self.cylinder.get_dist(), vel)
    }

    /// See [Component::measure()](`Component::measure()`)
    /// - `dist` is the maximum distance for the cylinder in mm
    /// - `vel` is the maximum linear velocity for the cylinder in mm per second
    /// - `set_dist` is the set distance for the cylinder in mm
    fn measure(&mut self, dist : f32, vel : f32, set_dist : f32, accuracy : u64) -> bool {
        self.cylinder.measure(dist, vel, set_dist, accuracy)
    }

    fn measure_async(&mut self, dist : f32, vel : f32, accuracy : u64) {
        self.cylinder.measure_async(dist, vel, accuracy)
    }

    // Distance
        fn get_dist(&self) -> f32 {
            self.dist_for_this(self.cylinder.get_dist())
        }

        fn dist_with_offset(&self, dist : f32) -> f32 {
            dist + self.offset_a.unwrap_or(0.0) + self.offset_b.unwrap_or(0.0)
        }

        fn dist_without_offset(&self, dist : f32) -> f32 {
            dist - self.offset_a.unwrap_or(0.0) - self.offset_b.unwrap_or(0.0)
        }

        /// See [Component::drive_abs](`Component::drive_abs()`)
        /// - `dist`is the angular distance to be moved (Unit radians)
        /// - `vel` is the cylinders extend velocity (Unit mm per second)
        fn drive_abs(&mut self, dist : f32, vel : f32) -> f32 {
            self.cylinder.drive_abs(self.dist_for_super(dist), vel)
        }

        fn drive_abs_async(&mut self, dist : f32, vel : f32) {
            self.cylinder.drive_abs_async(self.dist_for_super(dist), vel)
        }

        fn get_limit_dest(&self, gam : f32) -> LimitDest {
            match self.cylinder.get_limit_dest(self.dist_for_super(gam)) {
                LimitDest::Maximum(dist) => LimitDest::Maximum(self.dist_for_this(dist)),
                LimitDest::Minimum(dist) => LimitDest::Minimum(self.dist_for_this(dist)),
                other => other  
            }
        }
    // 
    
    // Forces
        fn apply_load_force(&mut self, force : f32) {
            self.cylinder.apply_load_force(force)
        }

        fn apply_load_inertia(&mut self, inertia : f32) {
            self.cylinder.apply_load_inertia(inertia)
        }
    // 
}