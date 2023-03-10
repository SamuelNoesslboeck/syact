use crate::{StepperConst, StepperCtrl, SyncComp};
use crate::comp::{GearBearing, Cylinder, CylinderTriangle};
use crate::data::LinkedData;
use crate::units::*;

#[test]
fn gear_bearing() -> Result<(), crate::Error> {
    const PIN_DIR : u8 = 27;
    const PIN_STEP : u8 = 19;

    let mut gear = GearBearing::new(StepperCtrl::new(StepperConst::MOT_17HE15_1504S, PIN_DIR, PIN_STEP), 0.1);
    gear.write_link(LinkedData::GEN);

    gear.drive_rel(Delta(0.5), Omega(0.5))?;

    Ok(())
}

#[test]
fn cylinder() -> Result<(), crate::Error> {
    const PIN_DIR : u8 = 22;
    const PIN_STEP : u8 = 11;

    let mut cylinder = Cylinder::new(StepperCtrl::new(StepperConst::MOT_17HE15_1504S, PIN_DIR, PIN_STEP), 1.27);
    cylinder.write_link(LinkedData::GEN);

    cylinder.drive_rel(Delta(25.0), Omega(25.0))?;

    Ok(())
}

#[test]
fn cylinder_bearing() -> Result<(), crate::Error> {
    const PIN_DIR : u8 = 22;
    const PIN_STEP : u8 = 11;

    let mut cyl_tri = CylinderTriangle::new(Cylinder::new(StepperCtrl::new(StepperConst::MOT_17HE15_1504S, PIN_DIR, PIN_STEP), 1.27), 200.0, 200.0);
    cyl_tri.write_link(LinkedData::GEN);

    cyl_tri.drive_rel(Delta(0.5), Omega(50.0))?;

    Ok(())
}