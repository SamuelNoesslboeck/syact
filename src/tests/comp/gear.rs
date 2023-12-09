use crate::prelude::*;

#[test]
#[ignore = "Value display, run manually ... "]
fn gear_values() {
    let mut stepper = Stepper::new_gen();
    stepper.set_config(StepperConfig::GEN);
    stepper.setup().unwrap();

    let mut gear = Gear::new(
        Stepper::new_gen(),
        2.0
    );
    gear.set_config(StepperConfig::GEN);
    gear.setup().unwrap();

    println!("Gear - Values");
}