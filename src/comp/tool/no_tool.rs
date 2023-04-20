use serde::{Serialize, Deserialize};

use glam::Vec3;

use crate::Tool;
use crate::units::*;

/// Placeholder tool
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NoTool { } // Empty struct

impl NoTool {
    /// Creates a new placeholder tool
    #[inline(always)]
    pub fn new() -> Self {
        Self { }
    }
}

impl Tool for NoTool {
    // Setup / Shutdown
        fn mount(&mut self) { }

        fn dismount(&mut self) { }
    // 

    // Stats
        fn get_json(&self) -> serde_json::Value {
            serde_json::to_value(self).unwrap() 
        }

        fn vec(&self) -> Vec3 {
            Vec3::ZERO
        }

        fn inertia(&self) -> Inertia {
            Inertia::ZERO
        }

        fn mass(&self) -> f32 {
            0.0
        }
    // 
}