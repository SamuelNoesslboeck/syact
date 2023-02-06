use serde::{Serialize, Deserialize};

use crate::{Component, StepperCtrl};
use crate::ctrl::SimpleMeas;
use crate::math::MathActor;

/// A bearing powered by a motor with a certain gear ratio
#[derive(Debug, Serialize, Deserialize)]
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

    // Json I/O
        fn to_json(&self) -> serde_json::Value {
            serde_json::to_value(self).unwrap()
        }
    //
}