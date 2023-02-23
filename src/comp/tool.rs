use core::any::type_name;
use core::fmt::Debug;

use serde::{Serialize, Deserialize};

use glam::Vec3;

// Tools
pub trait Tool : Debug
{
    // Actions
        fn activate(&self);

        fn rotate(&self);
    // 

    // Stats
        fn get_type_name(&self) -> &str {
            type_name::<Self>()
        }

        fn get_json(&self) -> serde_json::Value;

        /// Returns the characteristic vector of the tool
        fn get_vec(&self) -> Vec3;

        /// Returns the tool inhertia 
        fn get_inertia(&self) -> f32;

        /// Return the tool mass
        fn get_mass(&self) -> f32;
    //
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NoTool { } // Empty struct

impl NoTool 
{
    pub fn new() -> Self {
        Self { }
    }
}

impl Tool for NoTool
{
    // Actions
        fn activate(&self) { }

        fn rotate(&self) { }
    //

    // Stats
        fn get_json(&self) -> serde_json::Value {
            serde_json::to_value(self).unwrap() 
        }

        fn get_vec(&self) -> Vec3 {
            Vec3::new(0.0, 0.0, 0.0)
        }

        fn get_inertia(&self) -> f32 {
            0.0
        }

        fn get_mass(&self) -> f32 {
            0.0
        }
    // 
}

// // Tools
// #[derive(Serialize, Deserialize)]
// pub struct AxialBearing
// {
//     #[serde(skip)]
//     pub servo : ServoDriver,
//     pub length : f32,
//     pub mass : f32
// }

// impl AxialBearing {
//     pub fn new(pin_servo : u16, length : f32, mass : f32) -> Self {
//         AxialBearing {
//             servo: ServoDriver::new(ServoData::mg996r(), pin_servo),
//             length,
//             mass
//         }
//     }
// }

// impl Tool for AxialBearing
// {
//     // Actions
//         fn activate(&self) { }

//         fn rotate(&self) { }
//     //

//     // Stats
//         fn get_json(&self) -> serde_json::Value {
//             serde_json::to_value(self).unwrap()
//         }

//         fn get_vec(&self) -> Vec3 {
//             Vec3::new(0.0, self.length, 0.0)
//         }

//         fn get_inertia(&self) -> f32 {
//             self.mass * self.length.powi(2) / 12.0
//         }

//         fn get_mass(&self) -> f32 {
//             self.mass
//         }
//     // 
// }

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PencilTool
{
    pub length : f32,
    pub mass : f32
}

impl PencilTool {
    pub fn new(length : f32, mass : f32) -> Self {
        PencilTool {
            length,
            mass
        }
    }
}

impl Tool for PencilTool
{
    // Actions
        fn activate(&self) { }

        fn rotate(&self) { }
    // 

    // Stats
        fn get_json(&self) -> serde_json::Value {
            serde_json::to_value(self).unwrap()
        }

        fn get_vec(&self) -> Vec3 {
            Vec3::new(0.0, self.length, 0.0)
        }

        fn get_inertia(&self) -> f32 {
            self.mass * self.length.powi(2) / 12.0
        }

        fn get_mass(&self) -> f32 {
            self.mass
        }
    // 
}
//