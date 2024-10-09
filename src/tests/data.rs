use crate::data::StepperConst;

#[test]
#[ignore = "Value display, run manually ... "]
fn basic_plot() {
    let consts = StepperConst::GEN;

    let u = 12.0;
    
    println!("Stepper-Data");
    println!("- Velocity-Max: {}", consts.velocity_max(u));
}