use crate::{StepperCtrl, LinkedData, StepperConst, Component};

mod single_motor 
{
    use std::sync::Arc;

    use super::*;

    const PIN_DIR : u16 = 27;
    const PIN_STEP : u16 = 19;

    const STEPS : u64 = 200;
    const OMEGA : f32 = 10.0;

    #[test]
    fn step() {
        let mut ctrl = StepperCtrl::new(StepperConst::MOT_17HE15_1504S, PIN_DIR, PIN_STEP);
        ctrl.link(Arc::new(LinkedData { u: 12.0, s_f: 1.5 })); 
    
        ctrl.apply_load_inertia(0.000_1);
    
        println!("Doing single step ... ");
        ctrl.step(0.01, &crate::UpdateFunc::None);
        println!("Step done ... ");
    }
    
    #[test]
    fn steps() {
        let mut ctrl = StepperCtrl::new(StepperConst::MOT_17HE15_1504S, PIN_DIR, PIN_STEP);
        ctrl.link(Arc::new(LinkedData { u: 12.0, s_f: 1.5 })); 
    
        ctrl.apply_load_inertia(0.000_1);
        ctrl.apply_load_force(0.1);
    
        println!("Staring to move");
        ctrl.steps(STEPS, OMEGA, crate::ctrl::UpdateFunc::None);
        println!("{} with max speed {}rad/s done", STEPS, OMEGA);
    }
    // 
}