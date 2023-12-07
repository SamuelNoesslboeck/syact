use glam::Vec3;
use serde::{Serialize, Deserialize};

use crate::{Tool, Setup, Dismantle};
use crate::device::servo::Servo;
use crate::units::*;

use super::AxisTool;

/// A tool for adding an additional axis to the robot
#[derive(Debug, Serialize, Deserialize)]
pub struct AxialJoint {
    servo : Servo,

    /// Length of the tool
    pub length : f32,
    mass : Inertia
}

impl AxialJoint {
    /// Creates a new instance of an `AxialJoint`
    pub fn new(servo : Servo, length : f32, mass : Inertia) -> Self {
        Self {
            servo, 
            length,
            mass   
        }
    }
}

// Setup and Dismantle
impl Setup for AxialJoint {
    fn setup(&mut self) -> Result<(), crate::Error> {
        self.servo.setup()
    }
}

impl Dismantle for AxialJoint {
    fn dismantle(&mut self) -> Result<(), crate::Error> {
        self.servo.dismantle()
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

    fn vec(&self) -> Vec3 {
        Vec3::Y * self.length
    }

    fn inertia(&self) -> Inertia {
        self.length.powi(2) * self.mass / 12.0
    }

    fn mass(&self) -> f32 {
        self.mass.0
    }
}

impl AxisTool for AxialJoint {
    fn rotate_abs(&mut self, gamma : Gamma) {
        self.servo.set_gamma(Gamma(gamma.0 + self.servo.consts().gamma_max.0 / 2.0))
    }

    fn gamma(&self) -> Gamma {
        Gamma(self.servo.gamma().0 - self.servo.consts().gamma_max.0 / 2.0)
    }
}