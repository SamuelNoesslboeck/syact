use stepper_lib::{data::StepperData, ctrl::{PwmStepperCtrl, StepperCtrl}};


fn main() {
    let mut ctrl = PwmStepperCtrl::new(
        StepperData::mot_17he15_1504s(12.0), 
        3, 26);

    ctrl.sf = 10.0;
    // ctrl.data.t_s /= 2.0;
    ctrl.data.j_s = 0.000_1;


    // Test
    println!("Doing single step ... ");
    ctrl.step(0.01, &stepper_lib::UpdateFunc::None);
    println!("Step done ... ");
    // 
}