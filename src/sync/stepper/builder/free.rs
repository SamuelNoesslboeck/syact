use syunit::*;

use super::{StepperBuilder, ActuatorError};

pub struct FreeBuilder {

}

impl Iterator for FreeBuilder {
    type Item = Time;

    fn next(&mut self) -> Option<Self::Item> {
        todo!()
    }
}

impl StepperBuilder for FreeBuilder {
    // Getters
        fn step_angle(&self) -> U::Distance {
            todo!()
        }

        fn direction(&self) -> Direction {
            todo!()
        }
    // 
    
    // Setters
        fn set_overload_curret(&mut self, current : Option<f32>) -> Result<(), ActuatorError> {
            todo!()
        }
    //

    // Microsteps
        fn microsteps(&self) -> crate::MicroSteps {
            todo!()
        }

        fn set_microsteps(&mut self, microsteps : crate::MicroSteps) -> Result<(), ActuatorError> {
            todo!()
        }
    //

    // U::Velocity
        fn velocity_max(&self) -> Option<U::Velocity> {
            todo!()
        }

        fn set_velocity_max(&mut self, velocity_opt : Option<U::Velocity>) -> Result<(), ActuatorError> {
            todo!()
        }
    //

    // Acceleration
        fn acceleration_max(&self) -> Option<Acceleration> {
            todo!()
        }

        fn set_acceleration_max(&mut self, acceleration_opt : Option<Acceleration>) -> Result<(), ActuatorError> {
            todo!()
        }
    //

    // Jolt
        fn jolt_max(&self) -> Option<Jolt> {
            todo!()
        }

        fn set_jolt_max(&mut self, jolt_opt : Option<Jolt>) -> Result<(), ActuatorError> {
            todo!()
        }
    //

    // Drive mode
        fn drive_mode(&self) -> &super::DriveMode {
            todo!()
        }

        fn set_drive_mode<C : crate::prelude::StepperController>(&mut self, mode : super::DriveMode, ctrl : &mut C) -> Result<(), ActuatorError> {
            todo!()
        }
    //
}