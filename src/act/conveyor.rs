use serde::{Serialize, Deserialize};

use crate::{Stepper, SyncActuator, Setup};
use crate::act::parent::{ActuatorParent, RatioActuatorParent};
use crate::units::*;

/// A conveyor that uses a stepper as its motor
pub type StepperConveyor = Conveyor<Stepper>;

/// A simple conveyor powered by any kind of synchronous motor
#[derive(Debug)]
#[derive(Serialize, Deserialize)]
pub struct Conveyor<C : SyncActuator> {
    /// The parent component (driving the conveyor)
    device : C,

    /// Radius of the powered conveyor roll in millimeters
    pub r_roll : f32
}

impl<C : SyncActuator> Conveyor<C> {
    /// Creates a new instance of a conveyor
    /// - `device`: The parent component (driving the conveyor)
    /// - `r_roll` radius of the driving roll in millimeters
    pub fn new(device : C, r_roll : f32) -> Self {
        Self {
            device, 
            r_roll
        }
    }
}

impl<C : SyncActuator> Setup for Conveyor<C> {
    fn setup(&mut self) -> Result<(), crate::Error> {
        self.device.setup()
    }
}

// Parent
    impl<C : SyncActuator> ActuatorParent for Conveyor<C> {
        type Child = C;

        fn child(&self) -> &Self::Child {
            &self.device
        }

        fn child_mut(&mut self) -> &mut Self::Child {
            &mut self.device
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