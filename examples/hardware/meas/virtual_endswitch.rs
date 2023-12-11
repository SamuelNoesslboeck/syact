use syact::prelude::*;

fn main() -> Result<(), syact::Error> {
    // Initialize stepper
    let mut stepper = Stepper::new_gen();
    stepper.set_config(StepperConfig::GEN);
    stepper.setup()?;

    // Apply loads
    stepper.apply_inertia(Inertia(0.1));

    // Initialize switch
    let mut switch = VirtualEndSwitch::new(false, None);
    switch.setup()?;

    let vpin = switch.vpin.clone();
    stepper.add_interruptor(Box::new(switch));

    // Drive
    let gamma_0 = stepper.gamma();
    stepper.drive(sylo::Direction::CW, SpeedFactor::MAX)?;

    std::thread::sleep(core::time::Duration::from_millis(2000));
    
    vpin.store(true, core::sync::atomic::Ordering::Relaxed);

    stepper.await_inactive()?;

    println!("Movement done!");
    println!("Stats:");
    println!("- Delta: {}", stepper.gamma() - gamma_0);
    println!("- Gamma: {}", stepper.gamma());
    println!("- Reason: {:?}", stepper.intr_reason());

    Ok(())
}