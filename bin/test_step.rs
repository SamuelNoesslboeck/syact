use stepper_lib::{data::StepperData, ctrl::{StepperCtrl}};


fn main() {
    let mut ctrl = StepperCtrl::new(
        StepperData::mot_17he15_1504s(12.0, 1.5), 
        3, 26
    );

    ctrl.apply_load_j(0.000_01);

    // Test
    println!("Doing single step ... ");
    ctrl.step(0.01, &stepper_lib::UpdateFunc::None);
    println!("Step done ... ");
    // 
}