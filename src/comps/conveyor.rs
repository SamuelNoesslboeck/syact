use serde::{Serialize, Deserialize};

use crate::SyncActuator;
use crate::parent::{ActuatorParent, RatioActuatorParent};

use syunit::*;

/// A simple conveyor powered by any kind of synchronous motor
#[derive(Debug)]
#[derive(Serialize, Deserialize)]
pub struct Conveyor<C : SyncActuator> {
    /// The parent component (driving the conveyor)
    actuator : C,

    /// Radius of the powered conveyor roll in millimeters
    pub r_roll : f32
}

impl<C : SyncActuator> Conveyor<C> {
    /// Creates a new instance of a conveyor
    /// - `device`: The parent component (driving the conveyor)
    /// - `r_roll` radius of the driving roll in millimeters
    pub fn new(device : C, r_roll : f32) -> Self {
        Self {
            actuator: device, 
            r_roll
        }
    }
}

// Parent
    impl<C : SyncActuator> ActuatorParent for Conveyor<C> {
        type Child = C;

        fn child(&self) -> &Self::Child {
            &self.actuator
        }

        fn child_mut(&mut self) -> &mut Self::Child {
            &mut self.actuator
        }
    }

    impl<C : SyncActuator> RatioActuatorParent for Conveyor<C> {
        fn ratio(&self) -> f32 {
            self.r_roll
        }

        // Correct unit conversions
            fn force_for_child(&self, parent_force : Force) -> Force {
                parent_force * (self.ratio() / 1000.0)  // Ratio in millimeters => Conversion to meters for Newtonmeters
            }

            fn force_for_parent(&self, child_force : Force) -> Force {
                child_force / (self.ratio() / 1000.0)   // Ratio in millimeters => Conversion to meters for Newtonmeters 
            }

            fn inertia_for_child(&self, parent_inertia : Inertia) -> Inertia {
                parent_inertia * (self.ratio() / 1000.0) * (self.ratio() / 1000.0)
            }

            fn inertia_for_parent(&self, child_intertia : Inertia) -> Inertia {
                child_intertia / (self.ratio() / 1000.0) / (self.ratio() / 1000.0)
            }
        // 
    }
// 