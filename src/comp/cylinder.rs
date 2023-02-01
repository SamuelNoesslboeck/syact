use std::sync::Arc;

use serde::{Serialize, Deserialize};

use crate::{Component, LinkedData};
use crate::ctrl::{StepperCtrl, LimitType, LimitDest, SimpleMeas};
use crate::math::MathActor;

/// Cylinder component struct
#[derive(Serialize, Deserialize)]
pub struct Cylinder
{
    /// Data of the connected stepper motor
    pub ctrl : StepperCtrl,

    /// Distance traveled per rad (Unit mm)   \
    /// f_rte = pitch / (2pi)
    pub rte_ratio : f32,

    pub offset : Option<f32>
}

impl Cylinder
{
    /// Create a new cylinder instance
    pub fn new(ctrl : StepperCtrl, rte_ratio : f32, offset : Option<f32>) -> Self {
        return Cylinder {
            ctrl,
            rte_ratio,
            offset
        };
    }

    // Limits
        pub fn conv_limit(&self, limit : LimitType) -> LimitType {
            match limit {
                LimitType::Distance(dist) => LimitType::Angle(self.dist_for_super(dist)), 
                LimitType::Angle(_) => limit,
                _ => LimitType::None
            }
        }

        pub fn set_limit(&mut self, limit_max : LimitType, limit_min : LimitType) {
            self.ctrl.set_limit(self.conv_limit(limit_min), self.conv_limit(limit_max));
        }
    //
}

impl MathActor for Cylinder 
{
    fn accel_dyn(&self, vel : f32, pos : f32) -> f32 {
        self.dist_for_this(self.ctrl.accel_dyn(self.dist_for_super(vel), pos))
    }
}

impl SimpleMeas for Cylinder 
{
    fn init_meas(&mut self, pin_meas : u16) {
        self.ctrl.init_meas(pin_meas)
    }
}

impl Component for Cylinder 
{
    // Super
        fn super_comp(&self) -> Option<&dyn Component> {
            Some(&self.ctrl)
        }

        fn super_comp_mut(&mut self) -> Option<&mut dyn Component> {
            Some(&mut self.ctrl)
        }
        
        fn dist_for_super(&self, this_len : f32) -> f32 {
            this_len / self.rte_ratio
        }

        fn dist_for_this(&self, super_len : f32) -> f32 {
            super_len * self.rte_ratio
        }
    // 

    // Link
        fn link(&mut self, lk : Arc<LinkedData>) {
            self.ctrl.link(lk);    
        }
    //

    // JSON I/O
        fn to_json(&self) -> serde_json::Value {
            serde_json::to_value(self).unwrap()
        }
    // 

    // Position
        fn dist_with_offset(&self, dist : f32) -> f32 {
            dist + self.offset.unwrap_or(0.0)   
        }
        
        fn dist_without_offset(&self, dist : f32) -> f32 {
            dist - self.offset.unwrap_or(0.0)
        }

        fn get_limit_dest(&self, dist : f32) -> LimitDest {
            match self.ctrl.get_limit_dest(self.dist_for_super(dist)) {
                LimitDest::Minimum(ang) => LimitDest::Minimum(self.dist_for_this(ang)), 
                LimitDest::Maximum(ang) => LimitDest::Maximum(self.dist_for_this(ang)), 
                other => other
            }
        }
    //
}