use serde::{Serialize, Deserialize};

use crate::{Tool, Setup, Dismantle};
use crate::tool::{SimpleTool, AxisTool, Tongs};
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

// Setup and Dismantle
impl Setup for AxisTongs {
    fn setup(&mut self) -> Result<(), crate::Error> {
        self.axis.setup()?;
        self.tongs.setup()
    }
}

impl Dismantle for AxisTongs {
    fn dismantle(&mut self) -> Result<(), crate::Error> {
        self.axis.dismantle()?;
        self.tongs.dismantle()
    }
}

impl Tool for AxisTongs {
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