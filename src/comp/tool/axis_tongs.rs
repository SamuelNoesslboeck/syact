use serde::{Serialize, Deserialize};

use crate::Tool;
use crate::comp::tool::{SimpleTool, AxisTool, Tongs};
use crate::units::*;

use super::AxialJoint;

/// A combination of a pair of tongs and an axial joint, often used to control 
#[derive(Debug, Serialize, Deserialize)]
pub struct AxisTongs {
    /// The `AxialJoint` of the combination, adding an additional axis to the robot
    pub axis : AxialJoint,
    /// The `Tongs` of the combination
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
        #[inline]
        fn simple_tool(&self) -> Option<&dyn SimpleTool> {
            Some(&self.tongs)
        }

        #[inline]
        fn simple_tool_mut(&mut self) -> Option<&mut dyn SimpleTool> {
            Some(&mut self.tongs)
        }

        #[inline]
        fn axis_tool(&self) -> Option<&dyn AxisTool> {
            Some(&self.axis)   
        }

        #[inline]
        fn axis_tool_mut(&mut self) -> Option<&mut dyn AxisTool> {
            Some(&mut self.axis)
        }       
    //

    fn get_json(&self) -> serde_json::Value {
        serde_json::to_value(self).unwrap()
    }

    #[inline]
    fn vec(&self) -> glam::Vec3 {
        self.axis.vec() + self.tongs.vec()
    }

    #[inline]
    fn inertia(&self) -> Inertia {
        self.axis.inertia() + self.tongs.inertia()
    }

    #[inline]
    fn mass(&self) -> f32 {
        self.axis.mass() + self.tongs.mass()
    }
}