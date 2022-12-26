use crate::ctrl::{Component, StepperCtrl, LimitType, LimitDest};

use crate::comp::Cylinder;

pub struct CylinderTriangle 
{
    // Cylinder
    pub cylinder : Cylinder,

    // Triangle
    pub l_a : f32,
    pub l_b : f32
}

impl CylinderTriangle 
{
    pub fn new(cylinder : Cylinder, l_a : f32, l_b : f32) -> Self
    {
        let mut tri = CylinderTriangle {
            l_a, 
            l_b,
            cylinder 
        };

        tri.cylinder.write_length(l_a.max(l_b));

        return tri;
    }

    // Conversions
        /// Returns the cylinder length for the given angle gamma
        pub fn len_for_gam(&self, gam : f32) -> f32 {
            (self.l_a.powi(2) + self.l_b.powi(2) - 2.0 * self.l_a * self.l_b * gam.cos()).powf(0.5)
        }

        /// Returns the angle gamma for the given cylinder length _(len < (l_a + l_b))_
        pub fn gam_for_len(&self, len : f32) -> f32 {
            ((self.l_a.powi(2) + self.l_b.powi(2) - len.powi(2)) / 2.0 / self.l_a / self.l_b).acos()
        }
    //

    // Angle
        pub fn get_gam(&self) -> f32 {
            self.gam_for_len(self.cylinder.length())
        }

        pub fn set_gam(&mut self, gam : f32, v_max : f32) {
            self.cylinder.drive(self.len_for_gam(gam) - self.cylinder.length(), v_max);
        }

        pub fn set_gam_async(&mut self, gam : f32, v_max : f32) {
            self.cylinder.drive_async( self.len_for_gam(gam) - self.cylinder.length(), v_max);
        }
    //

    pub fn write_gam(&mut self, gam : f32) {
        self.cylinder.write_length(self.len_for_gam(gam))
    }

    pub fn measure(&mut self, max_dis : f32, v_max : f32, set_angle : f32, accuracy : u64) -> bool {
        self.cylinder.measure(max_dis, v_max,self.len_for_gam(set_angle), accuracy)
    }

    pub fn measure_async(&mut self, max_dis : f32, v_max : f32, accuracy : u64) {
        self.cylinder.measure_async(max_dis, v_max, accuracy);
    }
    
    // Limit
        pub fn set_limit(&mut self, limit_min : LimitType, limit_max : LimitType) {
            self.cylinder.set_limit(limit_max, limit_min)
        }

        pub fn get_limit_dest(&self, gam : f32) -> LimitDest {
            match self.cylinder.get_limit_dest(self.len_for_gam(gam)) {
                LimitDest::Maximum(dist) => LimitDest::Maximum(self.gam_for_len(dist)),
                LimitDest::Minimum(dist) => LimitDest::Minimum(self.gam_for_len(dist)),
                other => other  
            }
        }
    //
}

impl Component for CylinderTriangle {
    /// See [Component::drive](`Component::drive()`)
    /// - `dist`is the angular distance to be moved (Unit radians)
    /// - `vel` is the cylinders extend velocity (Unit mm per second)
    fn drive(&mut self, dist : f32, vel : f32) -> f32 {
        self.cylinder.drive(self.len_for_gam(dist) - self.cylinder.get_length(), vel)
    }

    /// See [Component::drive_async](`Component::drive_async()`)
    /// - `dist`is the angular distance to be moved (Unit radians)
    /// - `vel` is the cylinders extend velocity (Unit mm per second)
    fn drive_async(&mut self, dist : f32, vel : f32) {
        self.cylinder.drive_async(self.len_for_gam(dist) - self.cylinder.get_length(), vel)
    }

    fn measure(&mut self, dist : f32, vel : f32, set_dist : f32, accuracy : u64) -> bool {
        self.cylinder.measure(dist, vel, set_dist, accuracy)
    }

    fn measure_async(&mut self, dist : f32, vel : f32, accuracy : u64) {
        self.cylinder.measure_async(dist, vel, accuracy)
    }

    
}