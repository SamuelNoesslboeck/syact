//! #################
//! #   Movements   #
//! #################

use std::time::Instant;

use syact::math::movements::ptp_exact_unbuffered;
use syact::prelude::*;

fn main() -> Result<(), syact::Error> {
    println!("#################");
    println!("#   Movements   #");
    println!("#################");
 
    let mut group = [
        Stepper::new_gen(),
        Stepper::new_gen(),
        Stepper::new_gen(),
        Stepper::new_gen()
    ]; 

    group.write_data(CompData::GEN);
    group.setup()?;

    let deltas = [ Delta(10.0), Delta(20.0), Delta(15.0), Delta(25.0) ];
    let forces = [ Force(0.20), Force(0.10), Force(0.05), Force(0.10) ];
    let inertias = [ Inertia(0.01), Inertia(0.005), Inertia(0.03), Inertia(0.02) ];

    group.apply_forces(&forces);
    group.apply_inertias(&inertias);

    let inst = Instant::now();
    let times = ptp_exact_unbuffered(&mut group, deltas);
    println!("Times: {:?}; Calculation time: {}", times, inst.elapsed().as_secs_f32());

    Ok(())
}