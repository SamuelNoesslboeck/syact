//! Demonstration of different ways to implement a conveyor

use stepper_lib::prelude::*;

const PIN_DIR : u8 = 17;        // Pin of the directional signal
const PIN_STEP : u8 = 26;       // Pin of the step signal

#[cfg(all(feature = "predef", feature = "direct"))]
compile_error!("Please select only one of the approaches!");

// Helper sleep function
fn sleep(secs : f32) {
    std::thread::sleep(core::time::Duration::from_secs_f32(secs))
}

fn main() -> Result<(), stepper_lib::Error> {
    #[cfg(feature = "predef")]
    predefined::predefined()?;

    #[cfg(feature = "direct")]
    direct::direct_approach()?;

    Ok(())
}

/// Demonstration of the approach using predefined structs
#[cfg(feature = "predef")]
mod predefined {
    use super::*;

    // Define the radius of the powered conveyor roll as a constant with 5 millimeters
    const R_ROLL : f32 = 10.0;
    
    pub fn predefined() -> Result<(), stepper_lib::Error> {
        let mut conv = Conveyor::new(
            StepperCtrl::new(StepperConst::MOT_17HE15_1504S, PIN_DIR, PIN_STEP),        // The stepper motor
            R_ROLL
        );

        conv.write_link(LinkedData::GEN);
        conv.setup()?;
        
        // Apply a inertia to the conveyor (possible masses on the belt, 1.0kg estimated)
        conv.apply_inertia(Inertia(1.0));

        // Set the maximum speed of the conveyor to 40 millimeters per second
        conv.set_omega_max(Omega(200.0));
    
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

#[cfg(feature = "direct")]
mod direct {
    use super::*;

    // Define the radius of the powered conveyor roll as a constant with 5 millimeters
    const R_ROLL : f32 = 10.0;

    // Convert the linear conveyor speed to an angular speed of the motor
    fn omega_for_motor(omega_conv : Omega) -> Omega {
        omega_conv / R_ROLL
    }

    fn inertia_for_motor(inertia_conv : Inertia) -> Inertia {
        inertia_conv * R_ROLL * R_ROLL / 1_000_000.0      
        // R_ROLL has units millimeter, therefore a factor of 10^6 is required for conversion from kg to kgm^2
    }

    pub fn direct_approach() -> Result<(), stepper_lib::Error> {
        let mut ctrl = StepperCtrl::new(StepperConst::MOT_17HE15_1504S, PIN_DIR, PIN_STEP);
        ctrl.write_link(LinkedData::GEN);
        ctrl.setup()?;
        
        // Apply a inertia to the conveyor (possible masses on the belt)
        ctrl.apply_inertia(
            inertia_for_motor(Inertia(0.5))
        );

        // Set the maximum speed of the conveyor tp 40 millimeters per second
        ctrl.set_omega_max(
            omega_for_motor(Omega(200.0))
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