//! Demonstration of different ways to implement a conveyor

use syact::prelude::*;

const PIN_DIR : u8 = 17;        // Pin of the directional signal
const PIN_STEP : u8 = 26;       // Pin of the step signal

#[cfg(all(feature = "predef", feature = "direct"))]
compile_error!("Please select only one of the approaches!");

// Helper sleep function
fn sleep(secs : f32) {
    std::thread::sleep(core::time::Duration::from_secs_f32(secs))
}

fn main() -> Result<(), syact::Error> {
    #[cfg(feature = "predef")]
    predefined::predefined()?;

    #[cfg(feature = "direct")]
    direct::direct_approach()?;

    Ok(())
}

/// Demonstration of the approach using predefined structs
#[cfg(feature = "predef")]
mod predefined {
    use parent::*;

    // Define the radius of the powered conveyor roll as a constant with 5 millimeters
    const R_ROLL : f32 = 10.0;
    
    pub fn predefined() -> Result<(), syact::Error> {
        // First we crate our conveyor using a stepper motor and the radius of the roll that connects the belt to the motor
        let mut conv = Conveyor::new(
            Stepper::new(StepperConst::MOT_17HE15_1504S, PIN_DIR, PIN_STEP),        // The stepper motor
            R_ROLL
        );

        // Now we write the `CompData` to our component. The `CompData` is often data that is the same for 
        // all components, e.g. supply voltage
        conv.write_data(CompData {
            u: 12.0,        // Voltage
            s_f: 1.5        // Safety factor, the higher the factor, the safer is the stepper to not jump over steps,
                            // however the performance suffers from very high safety factors
        });

        // Setup all the neccessary stuff for a stepper motor
        // => Spawns the thread to execute async movements
        conv.setup()?;
        
        // Apply a inertia to the conveyor (possible masses on the belt, 1.0kg estimated)
        conv.apply_inertia(Inertia(1.0));

        // Set the maximum speed of the conveyor to 200 millimeters per second
        conv.set_omega_max(Omega(200.0));
    
        println!("Driving forward with 0.5 speed");
        conv.drive(Direction::CW, 0.5)?;        // Drive with 100 mm/s speed (50%, 0.5)
        conv.await_inactive()?;
    
        println!(" -> Reached speed!");
    
        sleep(1.0);
    
        println!("Driving forward with 0.8 speed");
        conv.drive(Direction::CW, 0.8)?;        // Drive with 160 mm/s speed (80%, 0.8)
        conv.await_inactive()?;
    
        println!(" -> Reached speed!");
    
        sleep(2.0);
    
        println!("Driving backwards with 0.2 speed");
        conv.drive(Direction::CCW, 0.2)?;       // Drive with 40 mm/s speed in the opposite direction (20%, 0.2)
        conv.await_inactive()?;
    
        println!("Reached speed!");
    
        sleep(1.0);
    
        println!("Finished!");

        Ok(())
    }
}

#[cfg(feature = "direct")]
mod direct {
    use parent::*;

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

    pub fn direct_approach() -> Result<(), syact::Error> {
        let mut ctrl = Stepper::new(StepperConst::MOT_17HE15_1504S, PIN_DIR, PIN_STEP);
        ctrl.write_data(CompData::GEN);
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
    use parent::*;

    pub struct CustomConveyor {
        ctrl : Stepper, 
        pub roll_radius : f32
    }

    // TODO: Finish custom comp
}