use crate::data::StepperConst;

#[test]
#[ignore = "Manual information display"]
fn basic_plot() {
    let consts = StepperConst::GEN;

    let u = 12.0;

    println!("Stepper-Data");
    println!("- Velocity-Max: {}", consts.omega_max(u));
}