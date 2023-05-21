//! Demonstration of different ways to implement a conveyor

use stepper_lib::prelude::*;

const PIN_DIR : u8 = 17;        // Pin of the directional signal
const PIN_STEP : u8 = 26;       // Pin of the step signal

// Helper sleep function
fn sleep(secs : f32) {
    std::thread::sleep(core::time::Duration::from_secs_f32(secs))
}


fn main() -> Result<(), stepper_lib::Error> {
    predefined::predefined()?;
    direct::direct_approach()?;

    Ok(())
}

/// Demonstration of the approach using predefined structs
mod predefined {
    use super::*;

    // Define the radius of the powered conveyor roll as a constant with 5 millimeters
    const R_ROLL : f32 = 5.0;
    
    pub fn predefined() -> Result<(), stepper_lib::Error> {
        let mut conv = Conveyor::new(
            StepperCtrl::new(StepperConst::MOT_17HE15_1504S, PIN_DIR, PIN_STEP),        // The stepper motor
            R_ROLL
        );

        conv.write_link(LinkedData::GEN);
        conv.setup()?;
        
        // Apply a inertia to the conveyor (possible masses on the belt)
        conv.apply_inertia(Inertia(0.01));

        // Set the maximum speed of the conveyor tp 40 millimeters per second
        conv.set_omega_max(Omega(40.0));
    
        println!("Driving forward with 0.5 speed");
        conv.drive(Direction::CW, 0.5)?;
        conv.await_inactive()?;
    
        println!(" -> Reached speed!");
    
        sleep(1.0);
    
        println!("Driving forward with 0.8 speed");
        conv.drive(Direction::CW, 0.8)?;
        conv.await_inactive()?;
    
        println!(" -> Reached speed!");
    
        sleep(2.0);
    
        println!("Driving backwards with 0.2 speed");
        conv.drive(Direction::CCW, 0.2)?;
        conv.await_inactive()?;
    
        println!("Reached speed!");
    
        sleep(1.0);
    
        println!("Finished!");

        Ok(())
    }
}

///
mod direct {
    use super::*;

    // Define the radius of the powered conveyor roll as a constant with 5 millimeters
    const R_ROLL : f32 = 5.0;

    // Convert the linear conveyor speed to an angular speed of the motor
    fn omega_for_motor(omega_conv : Omega) -> Omega {
        omega_conv / R_ROLL
    }

    pub fn direct_approach() -> Result<(), stepper_lib::Error> {
        let mut ctrl = StepperCtrl::new(StepperConst::MOT_17HE15_1504S, PIN_DIR, PIN_STEP);
        ctrl.write_link(LinkedData::GEN);
        ctrl.setup()?;
        
        // Apply a inertia to the conveyor (possible masses on the belt)
        ctrl.apply_inertia(Inertia(0.01));

        // Set the maximum speed of the conveyor tp 40 millimeters per second
        ctrl.set_omega_max(
            omega_for_motor(Omega(40.0))
        );
    
        println!("Driving forward with 0.5 speed");
        ctrl.drive(Direction::CW, 0.5)?;
        ctrl.await_inactive()?;
    
        println!(" -> Reached speed!");
    
        sleep(1.0);
    
        println!("Driving forward with 0.8 speed");
        ctrl.drive(Direction::CW, 0.8)?;
        ctrl.await_inactive()?;
    
        println!(" -> Reached speed!");
    
        sleep(2.0);
    
        println!("Driving backwards with 0.2 speed");
        ctrl.drive(Direction::CCW, 0.2)?;
        ctrl.await_inactive()?;
    
        println!("Reached speed!");
    
        sleep(1.0);
    
        println!("Finished!");

        Ok(())
    }
}

mod custom {
    use super::*;

    pub struct CustomConveyor {
        ctrl : StepperCtrl, 
        pub roll_radius : f32
    }

    // TODO: Finish custom comp
}