use stepper_lib::prelude::*;

const PIN_DIR : u8 = 27;        // Pin of the directional signal
const PIN_STEP : u8 = 19;       // Pin of the 

// Helper sleep function
fn sleep(secs : f32) {
    std::thread::sleep(core::time::Duration::from_secs_f32(secs))
}

fn main() -> Result<(), stepper_lib::Error> {
    let mut ctrl = StepperCtrl::new(StepperConst::GEN, PIN_DIR, PIN_STEP);
    ctrl.write_link(LinkedData::GEN);
    ctrl.setup()?;

    ctrl.apply_inertia(Inertia(0.01));

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