use serde::{Serialize, Deserialize};

use crate::{SyncActuator, Setup};
use crate::act::parent::{ActuatorParent, RatioActuatorParent};
use crate::act::stepper::Stepper;
use crate::units::*;

/// A linear axis using a stepper motor to power itself
pub type LinearStepper = LinearAxis<Stepper>;

/// A linear axis
#[derive(Debug, Serialize, Deserialize)]
pub struct LinearAxis<A : SyncActuator> {
    /// The child component driving the linear axis
    pub actuator : A,

    /// Distance traveled per rad (Unit mm)   \
    /// `f_rte = pitch / (2pi)`
    pub rte_ratio : f32
}

impl<A : SyncActuator> LinearAxis<A> {
    /// Create a new linear axis instance
    /// - `ctrl`: The parent component driving the linear axis
    /// - `rte_ratio`: (radius to extension ratio), millimeters travelled per radian the spindle rotated
    ///   - `f_rte = pitch / (2pi)`
    pub fn new(ctrl : A, rte_ratio : f32) -> Self {
        return LinearAxis {
            actuator: ctrl,
            rte_ratio
        };
    }
}

impl<A : SyncActuator> Setup for LinearAxis<A> {
    fn setup(&mut self) -> Result<(), crate::Error> {
        self.actuator.setup()
    }
}

// Parent
    impl<A : SyncActuator> ActuatorParent for LinearAxis<A> {
        type Child = A;

        fn child(&self) -> &Self::Child {
            &self.actuator
        }

        fn child_mut(&mut self) -> &mut Self::Child {
            &mut self.actuator
        }
    }

    impl<A : SyncActuator> RatioActuatorParent for LinearAxis<A> {
        fn ratio(&self) -> f32 {
            self.rte_ratio
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