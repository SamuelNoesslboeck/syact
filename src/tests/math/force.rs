use crate::{StepperConfig, StepperConst};
use syunit::*;

#[test]
fn torque_dyn() {
    let data = StepperConst::MOT_17HE15_1504S;

    // In stall, the output torque must be equal to the stall torque
    assert_eq!(data.torque_dyn(Velocity::ZERO, &StepperConfig::VOLT12_NO_OVERLOAD), data.torque_stall);
}