use core::any::type_name;
use core::fmt::Debug;

use glam::Vec3;

use crate::units::*;

// Tools
#[cfg(feature = "std")]
mod axial_joint;
#[cfg(feature = "std")]
pub use axial_joint::AxialJoint;

#[cfg(feature = "std")]
mod axis_tongs;
#[cfg(feature = "std")]
pub use axis_tongs::AxisTongs;

mod no_tool;
pub use no_tool::NoTool;

mod pencil_tool;
pub use pencil_tool::PencilTool;

#[cfg(feature = "std")]
mod tongs;
#[cfg(feature = "std")]
pub use tongs::Tongs;
//

// Tools
/// General tool trait
pub trait Tool : Debug {
    // Setup / Shutdown
        /// Mounts the component.
        /// 
        /// Comparable to a "setup" / "activate" function
        fn mount(&mut self);

        /// Dismounts the component.
        /// 
        /// Comparable to a "deactivate" function
        fn dismount(&mut self);
    // 

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
        fn get_vec(&self) -> Vec3;

        /// Returns the tool inhertia 
        fn get_inertia(&self) -> Inertia;

        /// Return the tool mass
        fn get_mass(&self) -> f32;
    //
}

// Subtools
    /// A trait for tools that add an additional axis for exact positioning 
    pub trait AxisTool : Tool {
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
    pub trait SimpleTool : Tool {
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
    pub trait SpindleTool : Tool {
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