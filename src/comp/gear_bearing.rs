use crate::{ctrl::{Component, StepperCtrl, LimitType, LimitDest}, UpdateFunc};

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

        /// Returns the omega for the motor from a given bearing omega
        pub fn omega_for_motor(&self, omega : f32) -> f32 {
            omega / self.ratio
        }

        /// Returns the angle for the motor from a given bearing angle
        pub fn ang_for_bear(&self, ang : f32) -> f32 {
            ang * self.ratio
        }
    //  

    pub fn get_pos(&self) -> f32 {
        self.ctrl.get_dist() * self.ratio
    }

    pub fn set_pos(&mut self, pos : f32, omega : f32) -> f32 {
        self.ctrl.drive(self.ang_for_motor(pos - self.get_pos()), self.omega_for_motor(omega), UpdateFunc::None)
    }

    pub fn set_pos_async(&mut self, pos : f32, omega : f32) {
        self.ctrl.drive_async(self.ang_for_motor(pos - self.get_pos()), self.omega_for_motor(omega), UpdateFunc::None);
    }

    pub fn measure(&mut self, max_angle : f32, omega : f32, set_pos : f32, accuracy : u64) -> bool {
        self.ctrl.measure(
            self.ang_for_motor(max_angle), 
            self.omega_for_motor(omega), 
            self.ang_for_motor(set_pos),
            accuracy
        )
    }

    pub fn measure_async(&mut self, max_angle : f32, omega : f32, accuracy : u64) {
        self.ctrl.measure_async(
            self.ang_for_motor(max_angle), 
            self.omega_for_motor(omega), 
            accuracy
        );
    }

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

    pub fn apply_load_j(&mut self, inertia : f32) {
        self.ctrl.apply_load_j(inertia * self.ratio);
    }

    pub fn apply_load_t(&mut self, torque : f32) {
        self.ctrl.apply_load_t(torque * self.ratio);
    }
}
