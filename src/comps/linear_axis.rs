use serde::{Serialize, Deserialize};
use syunit::metric::Millimeters;

use crate::SyncActuator;
use crate::parent::{ActuatorParent, RatioActuatorParent};

use syunit::*;

/// A linear axis
#[derive(Debug, Serialize, Deserialize)]
pub struct LinearAxis<A : SyncActuator> {
    /// The child actuator driving the linear axis
    pub actuator : A,
    /// The effective radius of the actuator, meaning the radius where
    /// 
    /// ```txt
    /// extension = radius * radians 
    /// ```
    /// 
    /// is true.
    pub effective_radius : Millimeters
}

impl<A : SyncActuator> LinearAxis<A> {
    /// Create a new linear axis driven by a tooth belt, with the driving gear having the `radius` given in [Millimeters]
    pub fn new_belt_axis(actuator : A, radius : Millimeters) -> Self {
        return LinearAxis {
            actuator,
            effective_radius: radius
        };
    }

    /// Create a new linear axis driven by a spindle with the given `pitch` in [Millimeters]
    pub fn new_spindle_axis(actuator : A, pitch : Millimeters) -> Self {
        return LinearAxis {
            actuator,
            effective_radius: pitch / 2.0 / core::f32::consts::PI   // Convert pitch to effective radius
        }
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
            self.effective_radius
        }
    }
// 