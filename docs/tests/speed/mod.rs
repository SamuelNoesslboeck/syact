use std::time::Instant;

use crate::prelude::*;

#[test]
#[ignore = "benchmark, run manually"]
fn torque_dyn_exact_speed() {
    let consts = StepperConst::GEN;

    let inst = Instant::now();

    for i in 0 .. 1_000_000 {
        force::torque_dyn(&consts, Omega(0.01 * (i as f32)), 12.0);
    }

    println!("Elapsed: {}s", inst.elapsed().as_secs_f32());
}

#[test]
#[ignore = "benchmark, run manually"]
fn torque_dyn_approx_speed() {
    let consts = StepperConst::GEN;

    let inst = Instant::now();
    let max_omega = consts.omega_max(12.0);

    for i in 0 .. 1_000_000 {
        force::torque_dyn_approx(&consts, Omega(0.01 * (i as f32)), max_omega);
    }

    println!("Elapsed: {}s", inst.elapsed().as_secs_f32());
}

#[test]
#[ignore = "benchmark, run manually"]
fn create_curve_speed() {
    let consts = StepperConst::GEN;
    let data = StepperConfig::GEN;
    let vars = CompVars::ZERO;

    let inst = Instant::now();

    let c = curve::create_simple_curve(&consts, &vars, &data, Delta(100_000.0), Omega(1_000.0), 1);

    println!(" => Created curve with {} nodes", c.len());
    println!("Elapsed: {}s", inst.elapsed().as_secs_f32());
}