use crate::{StepperConst, StepperCtrl, Component};
use crate::comp::GearBearing;
use crate::units::*;

#[test]
fn gear_bearing() {
    const PIN_DIR : u8 = 0;
    const PIN_STEP : u8 = 0;

    let mut gear = GearBearing::new(StepperCtrl::new(StepperConst::MOT_17HE15_1504S, PIN_DIR, PIN_STEP), 0.1);
    
    gear.drive_rel(Delta(0.5), Omega(0.5));
}