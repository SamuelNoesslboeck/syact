use crate::prelude::*;

#[test]
#[ignore = "Value display, run manually ... "]
fn gear_basic() {
    let mut gear = Gear::new(
        Stepper::new_gen(),
        2.0
    );
    gear.set_config(StepperConfig::GEN);
    gear.setup().unwrap();

    println!("Gear - Basics");
    gear.drive_rel(Delta(1.0), Factor::MAX).unwrap();
}