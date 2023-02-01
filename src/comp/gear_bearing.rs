use std::sync::Arc;
use serde::{Serialize, Deserialize};

use crate::{Component, LinkedData, StepperCtrl};
use crate::ctrl::{LimitType, LimitDest, SimpleMeas};
use crate::math::MathActor;

/// A bearing powered by a motor with a certain gear ratio
#[derive(Serialize, Deserialize)]
pub struct GearBearing 
{
    /// Steppercontrol for the motor of the bearing
    pub ctrl : StepperCtrl,
    
    /// Angle ration from motor to bearing (omega_b / omega_m)
    pub ratio : f32
}

impl GearBearing 
{
    pub fn new(ctrl : StepperCtrl, ratio : f32) -> Self {
        Self {
            ctrl,
            ratio
        }
    }

    // Limits
        pub fn set_limit(&mut self, limit_min : LimitType, limit_max : LimitType) {
            self.ctrl.set_limit(
                match limit_min {
                    LimitType::Angle(ang) => LimitType::Angle(self.dist_for_super(ang)), 
                    _ => LimitType::None
                }, match limit_max {
                    LimitType::Angle(ang) => LimitType::Angle(self.dist_for_super(ang)),
                    _ => LimitType::None
                }
            )
        }

        pub fn get_limit_dest(&self, gam : f32) -> LimitDest {
            match self.ctrl.get_limit_dest(self.dist_for_super(gam)) {
                LimitDest::Maximum(dist) => LimitDest::Maximum(self.dist_for_this(dist)),
                LimitDest::Minimum(dist) => LimitDest::Minimum(self.dist_for_this(dist)),
                other => other  
            }
        }
    //
}

impl SimpleMeas for GearBearing 
{
    fn init_meas(&mut self, pin_meas : u16) {
        self.ctrl.init_meas(pin_meas);
    }
}

impl MathActor for GearBearing
{
    fn accel_dyn(&self, vel : f32, pos : f32) -> f32 {
        self.dist_for_this(self.ctrl.accel_dyn(self.dist_for_super(vel), pos))
    }
}

impl Component for GearBearing 
{
    // Super
        fn super_comp(&self) -> Option<&dyn Component> {
            Some(&self.ctrl)
        }

        fn super_comp_mut(&mut self) -> Option<&mut dyn Component> {
            Some(&mut self.ctrl)
        }  

        /// Returns the angle for the motor from a given bearing angle
        fn dist_for_super(&self, this_len : f32) -> f32 {
            this_len / self.ratio
        }   

        /// Returns the angle for the motor from a given bearing angle
        fn dist_for_this(&self, super_len : f32) -> f32 {
            super_len * self.ratio
        }
    //

    // Link
        fn link(&mut self, lk : Arc<LinkedData>) {
            self.ctrl.link(lk);    
        }
    //

    // Json I/O
        fn to_json(&self) -> serde_json::Value {
            serde_json::to_value(self).unwrap()
        }
    //

    // Position
        fn get_limit_dest(&self, gam : f32) -> LimitDest {
            match self.ctrl.get_limit_dest(self.dist_for_super(gam)) {
                LimitDest::Maximum(dist) => LimitDest::Maximum(self.dist_for_this(dist)),
                LimitDest::Minimum(dist) => LimitDest::Minimum(self.dist_for_this(dist)),
                other => other  
            }
        }
    //
}