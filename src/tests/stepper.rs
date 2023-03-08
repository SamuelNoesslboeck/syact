mod single_motor 
{
    use crate::{StepperCtrl, StepperConst, Component};
    use crate::data::LinkedData;
    use crate::units::*;

    const PIN_DIR : u8 = 27;
    const PIN_STEP : u8 = 19;

    const STEPS : u64 = 50;
    const OMEGA : Omega = Omega(10.0);

    #[test]
    fn step() {
        let mut ctrl = StepperCtrl::new(StepperConst::MOT_17HE15_1504S, PIN_DIR, PIN_STEP);
        ctrl.link(LinkedData::GEN); 
    
        ctrl.apply_load_inertia(Inertia(0.000_1));
    
        println!("Doing single step ... ");
        ctrl.step(Time(0.01), &crate::ctrl::types::UpdateFunc::None);
        println!("Step done ... ");
    }
    
    #[test]
    fn steps() {
        let mut ctrl = StepperCtrl::new(StepperConst::MOT_17HE15_1504S, PIN_DIR, PIN_STEP);
        ctrl.link(LinkedData::GEN); 
    
        ctrl.apply_load_inertia(Inertia(0.4));
        ctrl.apply_load_force(Force(0.10));
    
        println!("Staring to move");
        ctrl.steps(STEPS, OMEGA, crate::ctrl::types::UpdateFunc::None);
        println!("{} with max speed {:?}rad/s done", STEPS, OMEGA);
    }
}