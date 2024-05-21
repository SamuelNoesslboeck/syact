use std::time::Instant;

use crate::math::movements::DefinedActuator;
use crate::prelude::*;
use crate::{Setup, Stepper, StepperConfig, SyncActuator};

#[tokio::test]
#[ignore = "Value display, run manually ... "]
async fn stepper_move_fixed_dist() {
    let mut stepper = Stepper::new_gen().unwrap();
    stepper.set_config(StepperConfig::GEN).unwrap();
    stepper.setup().unwrap();
    
    for _ in 0 .. 10 {
        let inst = Instant::now();

        let handle = tokio::spawn(   
            stepper.drive_rel(Delta(-10.0), Factor::MAX)
        );

        handle.await.unwrap().unwrap();
    
        let calc_time = stepper.ptp_time_for_distance(Gamma(0.0), Gamma(10.0));
    
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
    let mut ctrl = GenericPWM::new_gen().unwrap();
    let mut builder = ComplexBuilder::new(StepperConst::GEN).unwrap();
    builder.set_config(StepperConfig::GEN).unwrap();
    builder.set_microsteps(MicroSteps::from(16)).unwrap();

    dbg!(builder.velocity_cap());
    dbg!(&builder);

    builder.set_drive_mode(DriveMode::FixedDistance(Delta(0.02), Velocity::ZERO, Factor::MAX), &mut ctrl).unwrap();

    for (i, node) in builder.enumerate() {
        println!("{}: {}", i, node);
    }
}

#[test]
#[ignore = "Value display, run manually ... "]
fn builder_comparision() {
    const DELTA : Delta = Delta(0.3);

    let mut ctrl = GenericPWM::new_gen().unwrap();
    let mut builder = ComplexBuilder::new(StepperConst::GEN).unwrap();
    builder.set_config(StepperConfig::GEN).unwrap();
    builder.set_microsteps(MicroSteps::from(16)).unwrap();

    builder.set_drive_mode(DriveMode::FixedDistance(DELTA, Velocity::ZERO, Factor::MAX), &mut ctrl).unwrap();

    let mut time_sum = Time::ZERO;
    let pred = builder.ptp_time_for_distance(Gamma::ZERO, Gamma::ZERO + DELTA);

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
    let pred = builder.ptp_time_for_distance(Gamma::ZERO, Gamma::ZERO + DELTA);

    for (_i, node) in builder.enumerate() {
        // println!("{}: {}", _i, node);
        time_sum += node;
    }

    println!("Simple: Pred: {}, true: {}", pred, time_sum);
}

#[tokio::test]
#[ignore = "Value diplay, run manually ... "]
async fn gamma_distance() {
    let mut stepper = Stepper::new_gen().unwrap();  
    stepper.set_config(StepperConfig::GEN).unwrap();
    stepper.setup().unwrap();

    dbg!(stepper.gamma());
    stepper.drive_abs(Gamma(30.0), Factor::MAX).await.unwrap();
    dbg!(stepper.gamma());
    stepper.drive_abs(Gamma(10.0), Factor::MAX).await.unwrap();
    dbg!(stepper.gamma());
}