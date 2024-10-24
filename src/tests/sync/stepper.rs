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
    /// Helper stepper type for simulated stepper motors using Start-Stop control and simulated controllers
    pub type Stepper = StepperMotor<StartStopBuilder, SimulatedController>;
    /// Helper stepper type for simulated stepper motors using complex control and simulated controllers
    pub type ComplexStepper = StepperMotor<ComplexBuilder, SimulatedController>;

    impl Default for Stepper {
        fn default() -> Self {
            Self::new_advanced(SimulatedController::new(), StepperConst::MOT_17HE15_1504S, StepperConfig::VOLT12_NO_OVERLOAD).unwrap()
        }
    }

    impl Default for ComplexStepper {
        fn default() -> Self {
            Self::new_advanced(SimulatedController::new(), StepperConst::MOT_17HE15_1504S, StepperConfig::VOLT12_NO_OVERLOAD).unwrap()
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
                stepper.drive_rel_blocking(Radians(-10.0), Factor::MAX)
            );

            handle.join().unwrap().unwrap();
        });

        let elapsed_time = Seconds(inst.elapsed().as_secs_f32());
        let calc_time = stepper.ptp_time_for_distance(PositionRad(0.0), PositionRad(10.0));

        assert!(((elapsed_time / calc_time) - 1.0).abs() < PARAM_TIME_ACCURACY, "Time difference too large!\n -> Elapsed Time: {}\n -> Calculated time: {}", elapsed_time, calc_time);
    }
}

#[test]
fn abs_pos_distance() {
    let mut stepper = Stepper::default();

    dbg!(stepper.pos());
    stepper.drive_abs_blocking(PositionRad(30.0), Factor::MAX).unwrap();
    dbg!(stepper.pos());
    stepper.drive_abs_blocking(PositionRad(10.0), Factor::MAX).unwrap();
    dbg!(stepper.pos());
}