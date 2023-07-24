use std::time::Instant;

use syact::math::{HRLimitedStepBuilder, HRStepBuilder};
use syact::prelude::*;

fn main() -> Result<(), syact::Error> {
    let mut stepper = Stepper::new_gen();
    stepper.write_data(CompData::GEN);
    stepper.setup()?;

    let delta = Delta(10.0);

    let mut cur = HRLimitedStepBuilder::from_builder(
        HRStepBuilder::from_motor(&stepper, Omega::ZERO)
    );

    cur.set_omega_tar(stepper.omega_max())?;
    cur.set_steps_max(stepper.consts().steps_from_ang_abs(delta, stepper.micro()) - 1);

    let sum : f32 = cur.map(|t| t.0).sum();
    
    println!("Time calculated by builder: {}", sum);

    let inst = Instant::now();
    stepper.drive_rel(delta, 1.0)?;
    let elapsed = inst.elapsed().as_secs_f32();

    println!("Acutal time: {}", elapsed);

    Ok(())
}