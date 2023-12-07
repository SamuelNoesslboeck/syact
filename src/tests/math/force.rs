use crate::StepperConst;
use crate::math::force;
use crate::units::*;

#[test]
fn torque_dyn() {
    let data = StepperConst::GEN;

    // In stall, the output torque must be equal to the stall torque
    assert_eq!(force::torque_dyn(&data, Omega::ZERO, 12.0, data.current_max), data.torque_stall);
}