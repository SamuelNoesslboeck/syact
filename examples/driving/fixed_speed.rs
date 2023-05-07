use stepper_lib::{StepperCtrl, StepperConst, AsyncComp, SyncComp, Setup, LinkedData};
use stepper_lib::units::*;

const PIN_DIR : u8 = 27; 
const PIN_STEP : u8 = 19; 

fn sleep(secs : f32) {
    std::thread::sleep(core::time::Duration::from_secs_f32(secs))
}

fn main() -> Result<(), stepper_lib::Error> {
    let mut ctrl = StepperCtrl::new(StepperConst::GEN, PIN_DIR, PIN_STEP);
    ctrl.setup()?;
    ctrl.write_link(LinkedData::GEN);

    ctrl.apply_inertia(Inertia(0.1));

    ctrl.drive(stepper_lib::Direction::CW, 0.5)?;

    sleep(1.0);

    ctrl.drive(stepper_lib::Direction::CW, 0.8)?;

    sleep(2.0);

    ctrl.drive(stepper_lib::Direction::CCW, 0.2)?;

    sleep(1.0);

    Ok(())
}