use crate::{StepperConst, StepperCtrl, Component};
use crate::comp::{GearBearing, Cylinder, CylinderTriangle};
use crate::data::LinkedData;
use crate::units::*;

#[test]
fn gear_bearing() {
    const PIN_DIR : u8 = 27;
    const PIN_STEP : u8 = 19;

    let mut gear = GearBearing::new(StepperCtrl::new(StepperConst::MOT_17HE15_1504S, PIN_DIR, PIN_STEP), 0.1);
    gear.link(LinkedData::GEN);

    gear.drive_rel(Delta(0.5), Omega(0.5));
}

#[test]
fn cylinder() {
    const PIN_DIR : u8 = 22;
    const PIN_STEP : u8 = 11;

    let mut cylinder = Cylinder::new(StepperCtrl::new(StepperConst::MOT_17HE15_1504S, PIN_DIR, PIN_STEP), 1.27);
    cylinder.link(LinkedData::GEN);

    cylinder.drive_rel(Delta(25.0), Omega(25.0));
}

#[test]
fn cylinder_bearing() {
    const PIN_DIR : u8 = 22;
    const PIN_STEP : u8 = 11;

    let mut cyl_tri = CylinderTriangle::new(Cylinder::new(StepperCtrl::new(StepperConst::MOT_17HE15_1504S, PIN_DIR, PIN_STEP), 1.27), 200.0, 200.0);
    cyl_tri.link(LinkedData::GEN);

    cyl_tri.drive_rel(Delta(0.5), Omega(50.0));
}