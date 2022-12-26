use crate::{ctrl:: ServoDriver, Vec3, data::ServoData};

// Submodules
mod cylinder;
mod cylinder_triangle;
mod gear_bearing;

pub use cylinder::*;
pub use cylinder_triangle::*;
pub use gear_bearing::*;

// Tools
pub trait Tool
{
    // Actions
        fn activate(&self);

        fn rotate(&self);
    // 

    // Stats
        /// Returns the characteristic vector of the tool
        fn get_vec(&self) -> Vec3;

        /// Returns the tool inhertia 
        fn get_inertia(&self) -> f32;

        /// Return the tool mass
        fn get_mass(&self) -> f32;
    //
}

pub struct NoTool { } // Empty struct

impl NoTool 
{
    pub fn new() -> Self {
        Self { }
    }
}

impl Tool for NoTool
{
    fn activate(&self) { }

    fn rotate(&self) { }

    fn get_vec(&self) -> Vec3 {
        Vec3::new(0.0, 0.0, 0.0)
    }

    fn get_inertia(&self) -> f32 {
        0.0
    }

    fn get_mass(&self) -> f32 {
        0.0
    }
}

// Tools
pub struct AxialBearing
{
    pub servo : ServoDriver,
    pub length : f32,
    pub mass : f32
}

impl AxialBearing {
    pub fn new(pin_servo : u16, length : f32, mass : f32) -> Self {
        AxialBearing {
            servo: ServoDriver::new(ServoData::mg996r(), pin_servo),
            length,
            mass
        }
    }
}

impl Tool for AxialBearing
{
    fn activate(&self) { }

    fn rotate(&self) { }

    fn get_vec(&self) -> Vec3 {
        Vec3::new(0.0, self.length, 0.0)
    }

    fn get_inertia(&self) -> f32 {
        self.mass * self.length.powi(2) / 12.0
    }

    fn get_mass(&self) -> f32 {
        self.mass
    }
}

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
    fn activate(&self) { }

    fn rotate(&self) { }

    fn get_vec(&self) -> Vec3 {
        Vec3::new(0.0, self.length, 0.0)
    }

    fn get_inertia(&self) -> f32 {
        self.mass * self.length.powi(2) / 12.0
    }

    fn get_mass(&self) -> f32 {
        self.mass
    }
}
//