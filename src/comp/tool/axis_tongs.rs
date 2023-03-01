use serde::{Serialize, Deserialize};

use crate::Tool;

use crate::comp::tool::{SimpleTool, AxisTool, Tongs};

use super::AxialJoint;

#[derive(Debug, Serialize, Deserialize)]
pub struct AxisTongs {
    pub axis : AxialJoint,
    pub tong : Tongs
}

impl Tool for AxisTongs {
    // Upgrades
        fn simple_tool(&self) -> Option<&dyn SimpleTool> {
            Some(&self.tong)
        }

        fn simple_tool_mut(&mut self) -> Option<&mut dyn SimpleTool> {
            Some(&mut self.tong)
        }

        fn axis_tool(&self) -> Option<&dyn AxisTool> {
            Some(&self.axis)   
        }

        fn axis_tool_mut(&mut self) -> Option<&mut dyn AxisTool> {
            Some(&mut self.axis)
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