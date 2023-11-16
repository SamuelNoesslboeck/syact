use core::any::type_name;

use glam::Vec3;

use crate::{Dismantle, Setup};
use crate::units::*;

// Tools
    mod axial_joint;
    pub use axial_joint::AxialJoint;

    mod axis_tongs;
    pub use axis_tongs::AxisTongs;

    mod pencil_tool;
    pub use pencil_tool::PencilTool;

    mod relay;
    pub use relay::Relay;

    mod tongs;
    pub use tongs::Tongs;
//

// Tools
/// General tool trait
pub trait Tool : Setup + Dismantle {
    // Upgrade
        /// Upgrade the tool to a [SimpleTool] if possible, returns `None` otherwise
        fn simple_tool(&self) -> Option<&dyn SimpleTool> {
            None
        }

        /// Upgrade the tool to a [SimpleTool] if possible, returns `None` otherwise
        fn simple_tool_mut(&mut self) -> Option<&mut dyn SimpleTool> {
            None
        }

        /// Upgrade the tool to an [AxisTool] if possible, returns `None` otherwise
        fn axis_tool(&self) -> Option<&dyn AxisTool> {
            None
        }

        /// Upgrade the tool to an [AxisTool] if possible, returns `None` otherwise
        fn axis_tool_mut(&mut self) -> Option<&mut dyn AxisTool> {
            None
        }

        /// Upgrade the tool to an [SpindleTool] if possible, returns `None` otherwise
        fn spindle_tool(&self) -> Option<&dyn SpindleTool> {
            None
        }

        /// Upgrade the tool to an [SpindleTool] if possible, returns `None` otherwise
        fn spindle_tool_mut(&mut self) -> Option<&mut dyn SpindleTool> {
            None
        }
    //

    // Stats
        /// Returns the type name of the tool
        fn get_type_name(&self) -> &str {
            type_name::<Self>()
        }

        /// Returns the tool described as JSON-Object
        fn get_json(&self) -> serde_json::Value;

        /// Returns the characteristic vector of the tool
        fn vec(&self) -> Vec3;

        /// Returns the tool inhertia 
        fn inertia(&self) -> Inertia;

        /// Return the tool mass
        fn mass(&self) -> f32;
    //
}

// Subtools
    /// A trait for tools that add an additional axis for exact positioning 
    pub trait AxisTool {
        // Actions
            /// Rotate the axis to an exact position
            fn rotate_abs(&mut self, gamma : Gamma);
        //

        // State
            /// Return the exact position of the axis
            fn gamma(&self) -> Gamma;
        // 
    }

    /// A trait for tools that have a simple on/off or open/closed functionallity
    pub trait SimpleTool {
        // Actions
            /// Activates the tool
            fn activate(&mut self);

            /// Deactivates the tool
            fn deactivate(&mut self);

            /// Toggles the active status
            fn toggle(&mut self) {
                if self.is_active() {
                    self.deactivate()
                } else {
                    self.activate()
                }
            }
        // 

        // State    
            /// Returns the active-status of the tool
            fn is_active(&self) -> bool;
        // 
    }

    /// A trait for tools that have a spindle that can be driven into two directions
    pub trait SpindleTool {
        // Actions
            /// Activates the spindle, when `cw` is true, the motor drives clockwise
            fn activate(&mut self, cw : bool);

            /// Deactivates the spindle
            fn deactivate(&mut self);
        //

        // State
            /// Returns the active-status of the tool
            fn is_active(&self) -> Option<bool>;
        //
    }
//