use stepper_lib::{StepperCtrl, StepperData, UpdateFunc};
use std::f32::consts::PI;
 
fn main() {
    let ctrl = StepperCtrl::new(StepperData::mot_17he15_1504s(12.0, 1.5), 27, 19);
    ctrl.comms.drive_async(4.0 * PI, 2.0 * PI, UpdateFunc::None);

    ctrl.comms.await_inactive();
}