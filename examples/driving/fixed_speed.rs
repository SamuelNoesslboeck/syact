use stepper_lib::{StepperCtrl, StepperConst, AsyncComp, SyncComp, Setup, LinkedData};
use stepper_lib::units::*;

const PIN_DIR : u8 = 27; 
const PIN_STEP : u8 = 19; 

fn sleep(secs : f32) {
    spin_sleep::sleep(core::time::Duration::from_secs_f32(secs))
}

fn main() -> Result<(), stepper_lib::Error> {
    let mut ctrl = StepperCtrl::new(StepperConst::GEN, PIN_DIR, PIN_STEP);
    ctrl.setup()?;
    ctrl.setup_async();
    ctrl.write_link(LinkedData::GEN);

    ctrl.apply_inertia(Inertia(0.01));

    println!("Driving forward with 0.5 speed");
    ctrl.drive(stepper_lib::Direction::CW, 0.5)?;
    ctrl.await_inactive()?;

    sleep(1.0);

    println!("Driving forward with 0.8 speed");
    ctrl.drive(stepper_lib::Direction::CW, 0.8)?;
    ctrl.await_inactive()?;

    sleep(2.0);

    println!("Driving backwards with 0.2 speed");
    ctrl.drive(stepper_lib::Direction::CCW, 0.2)?;
    ctrl.await_inactive()?;

    println!("Finished!");

    sleep(1.0);

    Ok(())
}