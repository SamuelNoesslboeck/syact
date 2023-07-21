use glam::Vec3;
use serde::{Serialize, Deserialize};

use crate::{Tool, SimpleTool};
use crate::ctrl::servo::ServoDriver;
use crate::units::*;

/// A pair of controllable tongs with simple closed/open state switching
#[derive(Debug, Serialize, Deserialize)]
pub struct Tongs {
    servo : ServoDriver,

    /// Servo duty cycle percent for open state
    pub perc_open : f32,
    /// Servo duty cycle percent for closed state
    pub perc_close : f32,

    /// Length of the tongs
    pub length : f32,
    /// Mass of the tongs
    mass : Inertia
}

impl Tongs {
    /// Create a new `Tongs` instance
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

    /// Checks wheiter the tongs are closed 
    /// 
    /// # Note
    /// 
    /// This function just checks the set state by the software, it will not check the real state of the tongs!
    #[inline]
    pub fn closed(&self) -> bool {
        self.servo.perc() == self.perc_close
    }

    /// Opens the tongs by modifying the PWM Signal
    #[inline]
    pub fn open(&mut self) {
        self.servo.set_perc(self.perc_open)
    }

    /// Closes the tongs by modifying the PWM Signal
    #[inline]
    pub fn close(&mut self) {
        self.servo.set_perc(self.perc_close)
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

    fn vec(&self) -> Vec3 {
        Vec3::Y * self.length
    }

    fn inertia(&self) -> Inertia {
        self.mass * self.length.powi(2) / 12.0
    }

    fn mass(&self) -> f32 {
        self.mass.0
    }
}

impl SimpleTool for Tongs {
    /// Expands to `close()`
    fn activate(&mut self) {
        self.close()
    }

    /// Expands to `open()` 
    fn deactivate(&mut self) {
        self.open()
    }

    /// Expands to `is_closed()`
    fn is_active(&self) -> bool {
        self.closed()
    }
}
