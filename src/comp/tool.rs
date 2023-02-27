use core::any::type_name;
use core::fmt::Debug;

use glam::Vec3;

// Tools
mod axial_joint;
pub use axial_joint::AxialJoint;

mod axis_tongs;
pub use axis_tongs::AxisTongs;

mod no_tool;
pub use no_tool::NoTool;

mod pencil_tool;
pub use pencil_tool::PencilTool;

mod tongs;
pub use tongs::Tongs;
//

use crate::Gamma;

// Tools
pub trait Tool : Debug {
    // Upgrade
        fn simple_tool(&self) -> Option<&dyn SimpleTool> {
            None
        }

        fn simple_tool_mut(&mut self) -> Option<&mut dyn SimpleTool> {
            None
        }

        fn axis_tool(&self) -> Option<&dyn AxisTool> {
            None
        }

        fn axis_tool_mut(&mut self) -> Option<&mut dyn AxisTool> {
            None
        }
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

pub trait SimpleTool : Tool {
    // Actions
        fn activate(&mut self);
    // 

    // State 
        fn is_active(&self) -> bool;
    // 
}

pub trait AxisTool : Tool {
    // Actions
        fn rotate_abs(&mut self, gamma : Gamma);
    //

    // State
        fn gamma(&self) -> Gamma;
    // 
}