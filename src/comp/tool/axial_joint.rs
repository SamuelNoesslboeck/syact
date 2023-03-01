use glam::Vec3;
use serde::{Serialize, Deserialize};

use crate::{Tool, ctrl::servo::ServoDriver, Gamma};

use super::AxisTool;

#[derive(Debug, Serialize, Deserialize)]
pub struct AxialJoint {
    pub servo : ServoDriver,

    pub length : f32,
    pub mass : f32
}

impl AxialJoint {
    pub fn new(servo : ServoDriver, length : f32, mass : f32) -> Self {
        Self {
            servo, 
            length,
            mass   
        }
    }
}

impl Tool for AxialJoint {
    // Upgrades
        fn axis_tool(&self) -> Option<&dyn AxisTool> {
            Some(self)
        } 

        fn axis_tool_mut(&mut self) -> Option<&mut dyn AxisTool> {
            Some(self)
        }
    //

    fn get_json(&self) -> serde_json::Value {
        serde_json::to_value(self).unwrap()
    }

    fn get_vec(&self) -> Vec3 {
        Vec3::Y * self.length
    }

    fn get_inertia(&self) -> f32 {
        self.length.powi(2) * self.mass / 12.0
    }

    fn get_mass(&self) -> f32 {
        self.mass
    }
}

impl AxisTool for AxialJoint {
    fn rotate_abs(&mut self, gamma : Gamma) {
        self.servo.set_gamma(Gamma(gamma.0 - self.servo.data.gamma_max.0 / 2.0))
    }

    fn gamma(&self) -> Gamma {
        self.servo.gamma()
    }
}