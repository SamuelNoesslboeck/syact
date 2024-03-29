#[cfg(feature = "std")]
use serde::{Serialize, Deserialize}; 

use crate::{Stepper, SyncComp, Setup, CompData, AsyncComp};
use crate::comp::CompVars;
use crate::comp::stepper::StepperComp;
use crate::units::*;

/// A conveyor that uses a stepper as its motor
pub type StepperConveyor = Conveyor<Stepper>;

/// A simple conveyor powered by a stepper motor
#[derive(Debug)]
#[cfg_attr(feature = "std", derive(Serialize, Deserialize))]
pub struct Conveyor<C : SyncComp> {
    /// The parent component (driving the conveyor)
    ctrl : C,

    /// Radius of the powered conveyor roll in millimeters
    pub r_roll : f32
}

impl<C : SyncComp> Conveyor<C> {
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

impl<C : SyncComp> Setup for Conveyor<C> {
    fn setup(&mut self) -> Result<(), crate::Error> {
        self.ctrl.setup()
    }
}

impl<C : SyncComp> SyncComp for Conveyor<C> {
    fn vars<'a>(&'a self) -> &'a CompVars {
        self.ctrl.vars()
    }

    fn data<'a>(&'a self) -> &'a CompData {
        self.ctrl.data() 
    }

    // Super component
        fn parent_comp(&self) -> Option<&dyn SyncComp> {
            Some(&self.ctrl)
        }

        fn parent_comp_mut(&mut self) -> Option<&mut dyn SyncComp> {
            Some(&mut self.ctrl)
        }
    // 

    // Conversion
        fn gamma_for_parent(&self, this_gamma : Gamma) -> Gamma {
            this_gamma / self.r_roll
        }

        fn gamma_for_this(&self, parent_gamma : Gamma) -> Gamma {
            parent_gamma * self.r_roll
        }
    // 

    fn apply_force(&mut self, mut force : Force) {
        // Force in N to Nm, r_roll is in millimeters so factor of 1000.0 is required
        force = force * self.r_roll / 1000.0;   
        self.ctrl.apply_force(force)
    }

    fn apply_inertia(&mut self, mut inertia : Inertia) {
        // Inertia in kg to kg*m^2, r_roll is in millimeters so factor of 1000.0^2 => 1_000_000.0
        inertia = inertia * self.r_roll * self.r_roll / 1_000_000.0;    
        self.ctrl.apply_inertia(inertia)
    }
}

impl<C : AsyncComp + SyncComp> AsyncComp for Conveyor<C> {
    fn drive(&mut self, dir : crate::Direction, speed_f : f32) -> Result<(), crate::Error> {
        self.ctrl.drive(dir, speed_f)
    }

    fn dir(&self) -> crate::Direction {
        self.ctrl.dir()
    }

    fn speed_f(&self) -> f32 {
        self.ctrl.speed_f()
    }
}

impl<C : StepperComp> StepperComp for Conveyor<C> {
    #[inline]
    fn parent_stepper_comp(&self) -> Option<&dyn StepperComp> {
        Some(&self.ctrl)
    }

    #[inline]
    fn parent_stepper_comp_mut(&mut self) -> Option<&mut dyn StepperComp> {
        Some(&mut self.ctrl)
    }

    #[inline]
    fn motor(&self) -> &dyn crate::prelude::StepperMotor {
        self.ctrl.motor()
    }

    #[inline]
    fn motor_mut(&mut self) -> &mut dyn crate::prelude::StepperMotor {
        self.ctrl.motor_mut()
    }
}