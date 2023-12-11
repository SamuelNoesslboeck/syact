use crate::{StepperConst, ActuatorVars, StepperConfig};
use crate::data::MicroSteps;
use crate::math::HRLimitedStepBuilder;
use crate::units::*;

#[test]
fn hr_limited_step_builder() {
    let mut builder = HRLimitedStepBuilder::new(
        Omega::ZERO, 
        StepperConst::GEN, 
        ActuatorVars::ZERO, 
        StepperConfig::GEN, 
        StepperConst::GEN.omega_max(StepperConfig::GEN.voltage),
        MicroSteps::from(1)
    );

    builder.set_steps_max(1);

    let first = builder.next();
    
    assert!(first.is_some());
    assert_ne!(first, Some(Time::ZERO));
    assert!(builder.next().is_none());
}