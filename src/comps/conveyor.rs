//! ### Conveyor - General component
//! 
//! A conveyor powered by a synchronous actuator, for full description see [Conveyor]

#[cfg(feature = "serde")]
use serde::{Serialize, Deserialize};

use crate::SyncActuator;
use crate::parent::{ActuatorParent, RatioActuatorParent};

use syunit::*;
use syunit::metric::*;

/// ### Conveyor
/// 
/// A conveyor powered by a synchronous actuator ([SyncActuator])
#[derive(Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Conveyor<C : SyncActuator> {
    /// The child actuator, driving the conveyor, must be a [SyncActuator]
    actuator : C,

    /// Radius of the powered conveyor roll in [Millimeter]
    pub r_roll : Millimeters
}

impl<C : SyncActuator> Conveyor<C> {
    /// Creates a new instance of a [Conveyor]
    /// 
    /// - `actuator`: The child actuator, driving the conveyor, must be a [SyncActuator]
    /// - `r_roll` radius of the driving roll in [Millimeter]
    pub fn new(actuator : C, r_roll : Millimeters) -> Self {
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
        type Input = MetricMM;
        type Output = Rotary;
        type Ratio = Millimeters;


        fn ratio(&self) -> Self::Ratio {
            self.r_roll
        }
    }
// 