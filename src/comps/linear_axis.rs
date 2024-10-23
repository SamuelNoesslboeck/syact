use serde::{Serialize, Deserialize};
use syunit::metric::Millimeters;

use crate::SyncActuator;
use crate::parent::{ActuatorParent, RatioActuatorParent};

use syunit::*;

/// A linear axis
#[derive(Debug, Serialize, Deserialize)]
pub struct LinearAxis<A : SyncActuator> {
    /// The child component driving the linear axis
    pub actuator : A,

    pub radius : Millimeters
}

impl<A : SyncActuator> LinearAxis<A> {
    /// Create a new linear axis instance
    pub fn new(actuator : A, radius : Millimeters) -> Self {
        return LinearAxis {
            actuator,
            radius
        };
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
        type Input = MetricMM;
        type Output = Rotary;
        type Ratio = Millimeters;

        fn ratio(&self) -> Self::Ratio {
            self.radius
        }
    }
// 