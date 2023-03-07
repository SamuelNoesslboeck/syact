use serde::{Serialize, Deserialize};

use crate::Tool;

use crate::comp::tool::{SimpleTool, AxisTool, Tongs};

use super::AxialJoint;

#[derive(Debug, Serialize, Deserialize)]
pub struct AxisTongs {
    pub axis : AxialJoint,
    pub tongs : Tongs
}

impl Tool for AxisTongs {
    // Setup / Shutdown
        fn mount(&mut self) {
            self.axis.mount();
            self.tongs.mount();
        }

        fn dismount(&mut self) {
            self.axis.dismount();
            self.tongs.dismount();
        }
    // 

    // Upgrades
        fn simple_tool(&self) -> Option<&dyn SimpleTool> {
            Some(&self.tongs)
        }

        fn simple_tool_mut(&mut self) -> Option<&mut dyn SimpleTool> {
            Some(&mut self.tongs)
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
        self.axis.get_vec() + self.tongs.get_vec()
    }

    fn get_inertia(&self) -> f32 {
        self.axis.get_inertia() + self.tongs.get_inertia()
    }

    fn get_mass(&self) -> f32 {
        self.axis.get_mass() + self.tongs.get_inertia()
    }
}