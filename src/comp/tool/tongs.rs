use serde::{Serialize, Deserialize};

use crate::Tool;

use crate::comp::tool::SimpleTool;
use crate::ctrl::servo::ServoDriver;

#[derive(Debug, Serialize, Deserialize)]
pub struct Tongs {
    pub servo : ServoDriver
}

impl Tool for Tongs {
    // Upgrades
        fn simple_tool(&self) -> Option<&dyn SimpleTool> {
            Some(self)
        }

        fn simple_tool_mut(&mut self) -> Option<&mut dyn SimpleTool> {
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

impl SimpleTool for Tongs {
    fn activate(&mut self) {
        todo!()
    }

    fn is_active(&self) -> bool {
        todo!()
    }
}
