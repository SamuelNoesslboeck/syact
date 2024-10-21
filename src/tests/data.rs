use crate::data::StepperConst;

#[test]
#[ignore = "Value display, run manually ... "]
fn basic_plot() {
    let consts = StepperConst::MOT_17HE15_1504S;

    let u = 12.0;
    
    println!("Stepper-Data");
    println!("- U::Velocity-Max: {}", consts.velocity_max(u));
}