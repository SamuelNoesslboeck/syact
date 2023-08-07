use syact::prelude::*;

fn main() -> Result<(), syact::Error> {
    // Initialize stepper
    let mut stepper = Stepper::new_gen();
    stepper.write_data(CompData::GEN);
    stepper.setup()?;

    // Apply loads
    stepper.apply_inertia(Inertia(0.1));

    // Initialize switch
    let mut switch = VirtualEndSwitch::new(false);
    switch.setup()?;

    let vpin = switch.vpin.clone();
    stepper.add_interruptor(Box::new(switch));

    // Drive
    stepper.drive(Direction::CW, 1.0)?;

    std::thread::sleep(core::time::Duration::from_millis(2000));
    
    vpin.store(true, core::sync::atomic::Ordering::Relaxed);

    let delta = stepper.await_inactive()?;

    println!("Movement done!");
    println!("Stats:");
    println!("- Delta: {}", delta);
    println!("- Gamma: {}", stepper.gamma());
    println!("- Reason: {:?}", stepper.intr_reason());

    Ok(())
}