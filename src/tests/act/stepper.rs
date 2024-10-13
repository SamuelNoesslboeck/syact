use std::time::Instant;

use crate::{Stepper, StepperConfig, SyncActuator};
use crate::math::movements::DefinedActuator;
use crate::prelude::*;

#[test]
#[ignore = "Value display, run manually ... "]
fn stepper_move_fixed_dist() {
    let mut stepper = Stepper::new_gen().unwrap();
    stepper.set_config(StepperConfig::GEN).unwrap();

    for _ in 0 .. 10 {
        let inst = Instant::now();

        std::thread::scope(|s| {
            let handle = s.spawn(||   
                stepper.drive_rel(RelDist(-10.0), Factor::MAX)
            );

            handle.join().unwrap().unwrap();
        });

        let calc_time = stepper.ptp_time_for_distance(AbsPos(0.0), AbsPos(10.0));
    
        println!(" => Time elapsed: {}", inst.elapsed().as_secs_f32());
        println!(" => Expected: {}", calc_time);
        // println!("   | -> PWM: {}", stepper.)
    }
}

#[test]
#[ignore = "Value display, run manually ... "]
fn simple_builder() {
    let mut builder = StartStopBuilder::new(StepperConst::GEN).unwrap();
    builder.set_config(StepperConfig::GEN).unwrap();

    dbg!(&builder);

    // for (i, node) in builder.enumerate() {
    //     println!("{}: {}", i, node);
    // }
}

#[test]
#[ignore = "Value display, run manually ... "]
fn complex_builder() {
    let mut ctrl = GenericPWM::new_gen();
    let mut builder = ComplexBuilder::new(StepperConst::GEN).unwrap();
    builder.set_config(StepperConfig::GEN).unwrap();
    builder.set_microsteps(MicroSteps::from(16)).unwrap();

    dbg!(builder.velocity_cap());
    dbg!(&builder);

    builder.set_drive_mode(DriveMode::FixedDistance(RelDist(0.02), Velocity::ZERO, Factor::MAX), &mut ctrl).unwrap();

    for (i, node) in builder.enumerate() {
        println!("{}: {}", i, node);
    }
}

#[test]
#[ignore = "Value display, run manually ... "]
fn builder_comparision() {
    const DELTA : RelDist = RelDist(0.3);

    let mut ctrl = GenericPWM::new_gen();
    let mut builder = ComplexBuilder::new(StepperConst::GEN).unwrap();
    builder.set_config(StepperConfig::GEN).unwrap();
    builder.set_microsteps(MicroSteps::from(16)).unwrap();

    builder.set_drive_mode(DriveMode::FixedDistance(DELTA, Velocity::ZERO, Factor::MAX), &mut ctrl).unwrap();

    let mut time_sum = Time::ZERO;
    let pred = builder.ptp_time_for_distance(AbsPos::ZERO, AbsPos::ZERO + DELTA);

    for (_i, node) in builder.enumerate() {
        // println!("{}: {}", _i, node);
        time_sum += node;
    }

    println!("Complex: Pred: {}, true: {}", pred, time_sum);

    let mut builder = StartStopBuilder::new(StepperConst::GEN).unwrap();
    builder.set_config(StepperConfig::GEN).unwrap();
    builder.set_microsteps(MicroSteps::from(16)).unwrap();

    builder.set_drive_mode(DriveMode::FixedDistance(DELTA, Velocity::ZERO, Factor::MAX), &mut ctrl).unwrap();

    let mut time_sum = Time::ZERO;
    let pred = builder.ptp_time_for_distance(AbsPos::ZERO, AbsPos::ZERO + DELTA);

    for (_i, node) in builder.enumerate() {
        // println!("{}: {}", _i, node);
        time_sum += node;
    }

    println!("Simple: Pred: {}, true: {}", pred, time_sum);
}

#[tokio::test]
#[ignore = "Value diplay, run manually ... "]
async fn abs_pos_distance() {
    let mut stepper = Stepper::new_gen().unwrap();  
    stepper.set_config(StepperConfig::GEN).unwrap();

    dbg!(stepper.abs_pos());
    stepper.drive_abs(AbsPos(30.0), Factor::MAX).unwrap();
    dbg!(stepper.abs_pos());
    stepper.drive_abs(AbsPos(10.0), Factor::MAX).unwrap();
    dbg!(stepper.abs_pos());
}