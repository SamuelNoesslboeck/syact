use crate::prelude::*;
use crate as syact;

// #[derive(SyncCompGroup)]
// #[derive(StepperCompGroup)]
// struct TestGroup {
//     pub base : Stepper,
//     pub arm1 : Stepper
// }

// #[test]
// fn group_dyn() {
//     let group_arr_stat = [
//         Stepper::new_sim(StepperConst::GEN)
//     ];
//     let _group_arr_ref : &dyn SyncCompGroup<1> = &group_arr_stat;

//     let group_dyn_stat : [Box<dyn SyncComp>; 1] = [ 
//         Box::new(Stepper::new_sim(StepperConst::GEN))
//     ];
//     let _group_dyn_ref : &dyn SyncCompGroup<1> = &group_dyn_stat;

//     let test = TestGroup {
//         base: Stepper::new_sim(StepperConst::GEN),
//         arm1: Stepper::new_sim(StepperConst::GEN)
//     };

//     let _test_ref : &dyn SyncCompGroup<2> = &test;
// }

#[test]
#[cfg_attr(not(feature = "rasp"), ignore = "run manually when in simulation mode")]
fn gear_bearing() -> Result<(), crate::Error> {
    const PIN_DIR : u8 = 27;
    const PIN_STEP : u8 = 19;

    let mut gear = GearJoint::new(Stepper::new(StepperConst::MOT_17HE15_1504S, PIN_DIR, PIN_STEP), 0.1);
    gear.write_data(CompData::GEN);
    gear.setup()?;

    // gear.set_omega_max(Omega(0.5)); 
    gear.drive_rel(Delta(0.5), 1.0)?;

    Ok(())
}

#[test]
#[cfg_attr(not(feature = "rasp"), ignore = "run manually when in simulation mode")]
fn cylinder() -> Result<(), crate::Error> {
    const PIN_DIR : u8 = 22;
    const PIN_STEP : u8 = 11;

    let mut cylinder = Cylinder::new(Stepper::new(StepperConst::MOT_17HE15_1504S, PIN_DIR, PIN_STEP), 1.27);
    cylinder.write_data(CompData::GEN);
    cylinder.setup()?;
    

    cylinder.set_omega_max(Omega(38.0));
    cylinder.drive_rel(Delta(25.0), 1.0)?;

    Ok(())
}

#[test]
#[cfg_attr(not(feature = "rasp"), ignore = "run manually when in simulation mode")]
fn cylinder_tri() -> Result<(), crate::Error> {
    const PIN_DIR : u8 = 22;
    const PIN_STEP : u8 = 11;

    let mut cyl_tri = CylinderTriangle::new(
        Cylinder::new(Stepper::new(StepperConst::MOT_17HE15_1504S, PIN_DIR, PIN_STEP), 1.27), 200.0, 200.0);
    cyl_tri.write_data(CompData::GEN);
    cyl_tri.setup()?;

    cyl_tri.set_omega_max(Omega(25.0));
    cyl_tri.drive_rel(Delta(0.5), 1.0)?;

    Ok(())
}