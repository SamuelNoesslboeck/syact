use serde::{Serialize, Deserialize};

use glam::Vec3;

use crate::Tool;
use crate::units::*;

/// A tool representing a mounted pencil with a certain length
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PencilTool
{
    /// Length of the pencil
    pub length : f32,
    mass : Inertia
}

impl PencilTool {
    /// Creates a new instance of a `Pencil tool`
    pub fn new(length : f32, mass : Inertia) -> Self {
        PencilTool {
            length,
            mass
        }
    }
}

impl Tool for PencilTool {
    // Setup / Shutdown
        fn mount(&mut self) { }

        fn dismount(&mut self) { }
    // 

    // Stats
        fn get_json(&self) -> serde_json::Value {
            serde_json::to_value(self).unwrap()
        }

        fn vec(&self) -> Vec3 {
            Vec3::new(0.0, self.length, 0.0)
        }

        fn inertia(&self) -> Inertia {
            self.mass * self.length.powi(2) / 12.0
        }

        fn mass(&self) -> f32 {
            self.mass.0
        }
    // 
}