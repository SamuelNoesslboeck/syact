use crate::prelude::*;

#[test]
#[ignore = "Value display, run manually ... "]
fn simple_builder() {
    let builder = StartStopBuilder::new(StepperConst::MOT_17HE15_1504S, StepperConfig::VOLT12_NO_OVERLOAD).unwrap();

    for (i, node) in builder.enumerate() {
        println!("{}: {}", i, node);
    }
}


// #[test]
// #[ignore = "Value display, run manually ... "]
// fn complex_builder() {
//     let mut ctrl = SimulatedController::new_gen();
//     let mut builder = ComplexBuilder::new(StepperConst::GEN).unwrap();
//     builder.set_config(StepperConfig::GEN).unwrap();
//     builder.set_microsteps(MicroSteps::from(16)).unwrap();

//     dbg!(builder.velocity_cap());
//     dbg!(&builder);

//     builder.set_drive_mode(DriveMode::FixedDistance(RelDist(0.02), Velocity::ZERO, Factor::MAX), &mut ctrl).unwrap();

//     for (i, node) in builder.enumerate() {
//         println!("{}: {}", i, node);
//     }
// }

// #[test]
// #[ignore = "Value display, run manually ... "]
// fn builder_comparision() {
//     const DELTA : RelDist = RelDist(0.3);

//     let mut ctrl = SimulatedController::new_gen();
//     let mut builder = ComplexBuilder::new(StepperConst::GEN).unwrap();
//     builder.set_config(StepperConfig::GEN).unwrap();
//     builder.set_microsteps(MicroSteps::from(16)).unwrap();

//     builder.set_drive_mode(DriveMode::FixedDistance(DELTA, Velocity::ZERO, Factor::MAX), &mut ctrl).unwrap();

//     let mut time_sum = Time::ZERO;
//     let pred = builder.ptp_time_for_distance(AbsPos::ZERO, AbsPos::ZERO + DELTA);

//     for (_i, node) in builder.enumerate() {
//         // println!("{}: {}", _i, node);
//         time_sum += node;
//     }

//     println!("Complex: Pred: {}, true: {}", pred, time_sum);

//     let mut builder = StartStopBuilder::new(StepperConst::GEN).unwrap();
//     builder.set_config(StepperConfig::GEN).unwrap();
//     builder.set_microsteps(MicroSteps::from(16)).unwrap();

//     builder.set_drive_mode(DriveMode::FixedDistance(DELTA, Velocity::ZERO, Factor::MAX), &mut ctrl).unwrap();

//     let mut time_sum = Time::ZERO;
//     let pred = builder.ptp_time_for_distance(AbsPos::ZERO, AbsPos::ZERO + DELTA);

//     for (_i, node) in builder.enumerate() {
//         // println!("{}: {}", _i, node);
//         time_sum += node;
//     }

//     println!("Simple: Pred: {}, true: {}", pred, time_sum);
// }