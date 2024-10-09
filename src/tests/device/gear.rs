use crate::prelude::*;

#[tokio::test]
#[ignore = "Value display, run manually ... "]
async fn gear_basic() {
    let mut gear = Gear::new(
        Stepper::new_gen().unwrap(),
        2.0
    );
    gear.set_config(StepperConfig::GEN).unwrap();

    println!("Gear - Basics");
    gear.drive_rel(Delta(1.0), Factor::MAX).await.unwrap();
}