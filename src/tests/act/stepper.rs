use std::time::Instant;

use crate::math::movements::DefinedActuator;
use crate::prelude::StepperActuator;
use crate::{Setup, Stepper, StepperConfig, SyncActuator};

use syunit::*;

#[test]
fn stepper_move_fixed_dist() {
    let mut stepper = Stepper::new_gen();
    stepper.set_config(StepperConfig::GEN);
    stepper.setup().unwrap();
    
    let inst = Instant::now();

    stepper.drive_rel(Delta(-10.0), Factor::MAX).unwrap();

    let calc_time = stepper.ptp_time_for_distance(Gamma(0.0), Gamma(10.0));

    println!(" => Time elapsed: {}", inst.elapsed().as_secs_f32());
    println!(" => Expected: {}", calc_time);
    // println!("   | -> PWM: {}", stepper.)
}