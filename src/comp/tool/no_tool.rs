use serde::{Serialize, Deserialize};

use glam::Vec3;

use crate::Tool;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NoTool { } // Empty struct

impl NoTool {
    pub fn new() -> Self {
        Self { }
    }
}

impl Tool for NoTool {
    // Stats
        fn get_json(&self) -> serde_json::Value {
            serde_json::to_value(self).unwrap() 
        }

        fn get_vec(&self) -> Vec3 {
            Vec3::ZERO
        }

        fn get_inertia(&self) -> f32 {
            0.0
        }

        fn get_mass(&self) -> f32 {
            0.0
        }
    // 
}