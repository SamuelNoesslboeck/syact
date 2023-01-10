use crate::{ctrl::{Component, StepperCtrl, LimitType, LimitDest}};

/// Cylinder component struct
pub struct Cylinder
{
    /// Data of the connected stepper motor
    pub ctrl : StepperCtrl,

    /// Distance traveled per rad (Unit mm)   \
    /// f_rte = pitch / (2pi)
    pub rte_ratio : f32
}

impl Cylinder
{
    /// Create a new cylinder instance
    pub fn new(ctrl : StepperCtrl, rte_ratio : f32) -> Self {
        return Cylinder {
            ctrl,
            rte_ratio
        };
    }

    // Conversions
        /// Angle for extent
        pub fn dist_to_ang(&self, dist : f32) -> f32 {
            dist / self.rte_ratio
        }

        /// Extent for angle
        pub fn ang_to_dist(&self, ang : f32) -> f32 {
            ang * self.rte_ratio
        }

        /// Angular speed for linear velocity
        pub fn vel_to_omega(&self, vel : f32) -> f32 {
            vel / self.rte_ratio
        }

        /// Linear velocity for angular speed
        pub fn omega_to_vel(&self, omega : f32) -> f32 {
            omega * self.rte_ratio
        }
    //

    // Limits
        pub fn conv_limit(&self, limit : LimitType) -> LimitType {
            match limit {
                LimitType::Distance(dist) => LimitType::Angle(self.dist_to_ang(dist)), 
                LimitType::Angle(_) => limit,
                _ => LimitType::None
            }
        }

        pub fn set_limit(&mut self, limit_max : LimitType, limit_min : LimitType) {
            self.ctrl.set_limit(self.conv_limit(limit_min), self.conv_limit(limit_max));
        }
    //
}

impl Component for Cylinder 
{
    fn drive(&mut self, dist : f32, vel : f32) -> f32 {
        let res = self.ctrl.drive(self.dist_to_ang(dist), self.vel_to_omega(vel));
        self.ang_to_dist(res)
    }

    fn drive_async(&mut self, dist : f32, v_max : f32) {
        self.ctrl.drive_async(self.dist_to_ang(dist), self.vel_to_omega(v_max))
    }

    fn measure(&mut self, max_dis : f32, v_max : f32, set_len : f32, accuracy : u64) -> bool {
        self.ctrl.measure(
            self.dist_to_ang(max_dis), 
            self.vel_to_omega(v_max), 
            self.dist_to_ang(set_len), 
            accuracy
        )
    }

    fn measure_async(&mut self, max_dis : f32, v_max : f32, accuracy : u64) {
        self.ctrl.measure_async(self.dist_to_ang(max_dis), self.vel_to_omega(v_max),  accuracy);
    }

    // Position
        fn get_dist(&self) -> f32 {
            self.ang_to_dist(self.ctrl.get_dist())
        }

        fn drive_abs(&mut self, dist : f32, vel : f32) -> f32 {
            self.ctrl.drive_abs(self.dist_to_ang(dist), self.vel_to_omega(vel))
        }

        fn drive_abs_async(&mut self, dist : f32, vel : f32) {
            self.ctrl.drive_abs_async(self.dist_to_ang(dist), self.vel_to_omega(vel))
        }

        fn write_dist(&mut self, dis_c : f32) {
            self.ctrl.write_dist(self.dist_to_ang(dis_c));
        }

        fn get_limit_dest(&self, dist : f32) -> LimitDest {
            match self.ctrl.get_limit_dest(self.dist_to_ang(dist)) {
                LimitDest::Minimum(ang) => LimitDest::Minimum(self.ang_to_dist(ang)), 
                LimitDest::Maximum(ang) => LimitDest::Maximum(self.ang_to_dist(ang)), 
                other => other
            }
        }
    //

    // Loads
        fn accel_dyn(&self, vel : f32, pos : f32) -> f32 {
            self.omega_to_vel(self.ctrl.accel_dyn(self.vel_to_omega(vel), pos))
        }

        fn apply_load_inertia(&mut self, mass : f32) {
            self.ctrl.apply_load_inertia(mass * (self.rte_ratio / 1000.0).powi(2));
        }

        fn apply_load_force(&mut self, force : f32) {
            self.ctrl.apply_load_force(force * self.rte_ratio / 1000.0);
        }
    //
}