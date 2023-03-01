use crate::{comp::GearBearing, StepperConst, StepperCtrl, Component, Delta, Omega};

#[test]
fn gear_bearing() {
    const PIN_DIR : u16 = 0;
    const PIN_STEP : u16 = 0;

    let mut gear = GearBearing::new(StepperCtrl::new(StepperConst::MOT_17HE15_1504S, PIN_DIR, PIN_STEP), 0.1);
    
    gear.drive_rel(Delta(0.5), Omega(0.5));
}