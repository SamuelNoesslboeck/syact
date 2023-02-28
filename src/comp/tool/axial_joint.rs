use serde::{Serialize, Deserialize};

use crate::{Tool, ctrl::servo::ServoDriver, Gamma};

use super::AxisTool;

#[derive(Debug, Serialize, Deserialize)]
pub struct AxialJoint {
    pub servo : ServoDriver
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

    fn get_vec(&self) -> glam::Vec3 {
        todo!()
    }

    fn get_inertia(&self) -> f32 {
        todo!()
    }

    fn get_mass(&self) -> f32 {
        todo!()
    }
}

impl AxisTool for AxialJoint {
    fn rotate_abs(&mut self, _ : Gamma) {
        todo!()
    }

    fn gamma(&self) -> Gamma {
        todo!()
    }
}