//! ### Conveyor - General component
//! 
//! A conveyor powered by a synchronous actuator, for full description see [Conveyor]

#[cfg(feature = "serde")]
use serde::{Serialize, Deserialize};

use crate::SyncActuator;
use crate::parent::{ActuatorParent, RatioActuatorParent};

use syunit::*;
use syunit::metric::Millimeter;

/// ### Conveyor
/// 
/// A conveyor powered by a synchronous actuator ([SyncActuator])
#[derive(Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Conveyor<C : SyncActuator> {
    /// The child actuator, driving the conveyor, must be a [SyncActuator]
    actuator : C,

    /// Radius of the powered conveyor roll in [Millimeter]
    pub r_roll : Millimeter
}

impl<C : SyncActuator> Conveyor<C> {
    /// Creates a new instance of a [Conveyor]
    /// 
    /// - `actuator`: The child actuator, driving the conveyor, must be a [SyncActuator]
    /// - `r_roll` radius of the driving roll in [Millimeter]
    pub fn new(actuator : C, r_roll : Millimeter) -> Self {
        Self {
            actuator, 
            r_roll
        }
    }
}

// ######################################
// #    Actuator-Parent relationship    #
// ######################################
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
            self.r_roll.0
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