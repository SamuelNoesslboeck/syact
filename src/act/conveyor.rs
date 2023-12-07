use serde::{Serialize, Deserialize};
use sylo::Direction; 

use crate::{Stepper, SyncActuator, Setup, AsyncActuator};
use crate::act::parent::{ActuatorParent, RatioActuatorParent};
use crate::units::*;

/// A conveyor that uses a stepper as its motor
pub type StepperConveyor = Conveyor<Stepper>;

/// A simple conveyor powered by any kind of synchronous motor
#[derive(Debug)]
#[derive(Serialize, Deserialize)]
pub struct Conveyor<C : SyncActuator> {
    /// The parent component (driving the conveyor)
    ctrl : C,

    /// Radius of the powered conveyor roll in millimeters
    pub r_roll : f32
}

impl<C : SyncActuator> Conveyor<C> {
    /// Creates a new instance of a conveyor
    /// - `ctrl`: The parent component (driving the conveyor)
    /// - `r_roll` radius of the driving roll in millimeters
    pub fn new(ctrl : C, r_roll : f32) -> Self {
        Self {
            ctrl, 
            r_roll
        }
    }
}

impl<C : SyncActuator> Setup for Conveyor<C> {
    fn setup(&mut self) -> Result<(), crate::Error> {
        self.ctrl.setup()
    }
}

// Parent
    impl<C : SyncActuator> ActuatorParent for Conveyor<C> {
        type Child = C;

        fn child(&self) -> &Self::Child {
            &self.ctrl
        }

        fn child_mut(&mut self) -> &Self::Child {
            &mut self.ctrl
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

impl<C : AsyncActuator<Duty = f32> + SyncActuator> AsyncActuator for Conveyor<C> {
    type Duty = f32;

    fn drive(&mut self, dir : Direction, speed_f : f32) -> Result<(), crate::Error> {
        self.ctrl.drive(dir, speed_f)
    }

    fn dir(&self) -> Direction {
        self.ctrl.dir()
    }

    fn speed(&self) -> f32 {
        self.ctrl.speed()
    }
}