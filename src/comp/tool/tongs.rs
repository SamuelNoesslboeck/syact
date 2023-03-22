use glam::Vec3;
use serde::{Serialize, Deserialize};

use crate::Tool;
use crate::comp::tool::SimpleTool;
use crate::ctrl::servo::ServoDriver;
use crate::units::*;


#[derive(Debug, Serialize, Deserialize)]
pub struct Tongs {
    servo : ServoDriver,

    pub perc_open : f32,
    pub perc_close : f32,

    pub length : f32,
    pub mass : Inertia
}

impl Tongs {
    pub fn new(servo : ServoDriver, perc_open : f32, perc_close : f32, length : f32, mass : Inertia) -> Self {
        let mut tongs = Self {
            servo,
            perc_open,
            perc_close,
            length,
            mass
        };

        tongs.deactivate();

        tongs
    }
}

impl Tool for Tongs {
    // Setup / Shutdown
        fn mount(&mut self) {
            self.servo.start();
        }

        fn dismount(&mut self) {
            self.servo.stop();
        }
    // 

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

    fn get_vec(&self) -> Vec3 {
        Vec3::Y * self.length
    }

    fn get_inertia(&self) -> Inertia {
        self.mass * self.length.powi(2) / 12.0
    }

    fn get_mass(&self) -> f32 {
        self.mass.0
    }
}

impl SimpleTool for Tongs {
    fn activate(&mut self) {
        self.servo.set_perc(self.perc_close)
    }

    fn deactivate(&mut self) {
        self.servo.set_perc(self.perc_open)
    }

    fn is_active(&self) -> bool {
        self.servo.perc() == self.perc_close
    }
}
