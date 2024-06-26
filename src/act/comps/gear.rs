use serde::{Serialize, Deserialize};

use crate::{SyncActuator, Setup};
use crate::act::parent::{ActuatorParent, RatioActuatorParent};

/// A gear component
/// 
/// # Gears
/// 
/// 
#[derive(Debug, Serialize, Deserialize)]
pub struct Gear<C : SyncActuator> {
    /// Steppercontrol for the motor of the bearing
    pub ctrl : C,
    
    /// Angle ration from motor to bearing (velocity_b / velocity_m)
    pub ratio : f32
}

impl<C : SyncActuator> Gear<C> {
    /// Creates a new `Gearbearing`
    /// - `device`: The parent component driving the cylinder
    pub fn new(ctrl : C, ratio : f32) -> Self {
        Self {
            ctrl,
            ratio
        }
    }
}

// Parent
    impl<C : SyncActuator> Setup for Gear<C> {
        fn setup(&mut self) -> Result<(), crate::Error> {
            self.ctrl.setup() 
        }
    }

    impl<C : SyncActuator> ActuatorParent for Gear<C> {
        type Child = C;

        fn child(&self) -> &Self::Child {
            &self.ctrl
        }

        fn child_mut(&mut self) -> &mut Self::Child {
            &mut self.ctrl
        }
    }

    impl<C : SyncActuator> RatioActuatorParent for Gear<C> {
        fn ratio(&self) -> f32 {
            self.ratio
        }
    }
// 