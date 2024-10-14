use crate::StepperConst;
use crate::math::force;
use syunit::*;

#[test]
fn torque_dyn() {
    let data = StepperConst::MOT_17HE15_1504S;

    // In stall, the output torque must be equal to the stall torque
    assert_eq!(force::torque_dyn(&data, Velocity::ZERO, 12.0, None), data.torque_stall);
}