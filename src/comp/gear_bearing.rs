use crate::{Component, StepperCtrl, ctrl::{LimitType, LimitDest}};

/// A bearing powered by a motor with a certain gear ratio
pub struct GearBearing 
{
    /// Steppercontrol for the motor of the bearing
    pub ctrl : StepperCtrl,
    
    /// Angle ration from motor to bearing (omega_b / omega_m)
    pub ratio : f32
}

impl GearBearing 
{
    // Converstions
        /// Returns the angle for the motor from a given bearing angle
        pub fn ang_for_motor(&self, ang : f32) -> f32 {
            ang / self.ratio
        }

        /// Returns the omega (angluar velocity) for the motor from a given bearing omega
        pub fn vel_for_motor(&self, omega : f32) -> f32 {
            omega / self.ratio
        }

        /// Returns the angle for the motor from a given bearing angle
        pub fn ang_for_bear(&self, ang : f32) -> f32 {
            ang * self.ratio
        }
    //  

    // Limits
        pub fn set_limit(&mut self, limit_min : LimitType, limit_max : LimitType) {
            self.ctrl.set_limit(
                match limit_min {
                    LimitType::Angle(ang) => LimitType::Angle(self.ang_for_motor(ang)), 
                    _ => LimitType::None
                }, match limit_max {
                    LimitType::Angle(ang) => LimitType::Angle(self.ang_for_motor(ang)),
                    _ => LimitType::None
                }
            )
        }

        pub fn get_limit_dest(&self, gam : f32) -> LimitDest {
            match self.ctrl.get_limit_dest(self.ang_for_motor(gam)) {
                LimitDest::Maximum(dist) => LimitDest::Maximum(self.ang_for_bear(dist)),
                LimitDest::Minimum(dist) => LimitDest::Minimum(self.ang_for_bear(dist)),
                other => other  
            }
        }
    //
}

impl Component for GearBearing 
{
    fn drive(&mut self, dist : f32, vel : f32) -> f32 {
        self.ctrl.drive(self.ang_for_motor(dist), self.vel_for_motor(vel))
    }

    fn drive_async(&mut self, dist : f32, vel : f32) {
        self.ctrl.drive_async(self.ang_for_motor(dist), vel)
    }

    fn measure(&mut self, dist : f32, vel : f32, set_dist : f32, accuracy : u64) -> bool {
        self.ctrl.measure(self.ang_for_motor(dist), self.vel_for_motor(vel), self.ang_for_motor(set_dist), accuracy)
    }

    fn measure_async(&mut self, dist : f32, vel : f32, accuracy : u64) {
        self.ctrl.measure_async(self.ang_for_motor(dist), self.vel_for_motor(vel), accuracy)
    }

    // Position
        fn get_dist(&self) -> f32 {
            self.ctrl.get_dist() * self.ratio
        }

        fn drive_abs(&mut self, pos : f32, omega : f32) -> f32 {
            self.ctrl.drive_abs(self.ang_for_motor(pos), self.vel_for_motor(omega))
        }
        
        fn drive_abs_async(&mut self, dist : f32, vel : f32) {
            self.ctrl.drive_abs_async(self.ang_for_motor(dist), vel)
        }

        fn write_dist(&mut self, dist : f32) {
            self.ctrl.write_dist(self.ang_for_motor(dist))
        }
    //

    // Forces
        fn accel_dyn(&self, vel : f32) -> f32 {
            self.ctrl.accel_dyn(self.vel_for_motor(vel))
        }

        fn apply_load_force(&mut self, force : f32) {
            self.ctrl.apply_load_force(force * self.ratio);
        }

        fn apply_load_inertia(&mut self, inertia : f32) {
            self.ctrl.apply_load_inertia(inertia * self.ratio);
        }
    //
}