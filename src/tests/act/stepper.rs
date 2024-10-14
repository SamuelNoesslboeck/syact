use std::time::Instant;

use crate::prelude::*;
use crate::tests::PARAM_TIME_ACCURACY;

// ####################
// #    SUBMODULES    #
// ####################
    mod builder;

    pub mod ctrl;
    pub use ctrl::SimulatedController;
//

// #######################
// #    Stepper-Types    #
// #######################
    pub type Stepper = StepperMotor<StartStopBuilder, SimulatedController>;
    pub type ComplexStepper = StepperMotor<ComplexBuilder, SimulatedController>;

    impl Default for Stepper {
        fn default() -> Self {
            Self::new(SimulatedController::new(), StepperConst::MOT_17HE15_1504S, StepperConfig::VOLT12_NO_OVERLOAD).unwrap()
        }
    }

    impl Default for ComplexStepper {
        fn default() -> Self {
            Self::new(SimulatedController::new(), StepperConst::MOT_17HE15_1504S, StepperConfig::VOLT12_NO_OVERLOAD).unwrap()
        }
    }
// 

#[test]
fn stepper_move_fixed_dist() {
    let mut stepper = Stepper::default();

    for _ in 0 .. 10 {
        let inst = Instant::now();

        std::thread::scope(|s| {
            let handle = s.spawn(||   
                stepper.drive_rel(RelDist(-10.0), Factor::MAX)
            );

            handle.join().unwrap().unwrap();
        });

        let elapsed_time = Time(inst.elapsed().as_secs_f32());
        let calc_time = stepper.ptp_time_for_distance(AbsPos(0.0), AbsPos(10.0));

        assert!(((elapsed_time / calc_time) - 1.0).abs() < PARAM_TIME_ACCURACY, "Time difference too large!\n -> Elapsed Time: {}\n -> Calculated time: {}", elapsed_time, calc_time);
    }
}

#[test]
fn abs_pos_distance() {
    let mut stepper = Stepper::default();

    dbg!(stepper.abs_pos());
    stepper.drive_abs(AbsPos(30.0), Factor::MAX).unwrap();
    dbg!(stepper.abs_pos());
    stepper.drive_abs(AbsPos(10.0), Factor::MAX).unwrap();
    dbg!(stepper.abs_pos());
}