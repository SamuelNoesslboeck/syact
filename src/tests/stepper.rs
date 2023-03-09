mod single_motor 
{
    use core::f32::consts::PI;

    use crate::{StepperCtrl, StepperConst, SyncComp};
    use crate::data::LinkedData;
    use crate::units::*;

    const PIN_DIR : u8 = 27;
    const PIN_STEP : u8 = 19;

    const DELTA : Delta = Delta(2.0 * PI);
    const OMEGA : Omega = Omega(10.0);

    #[test]
    fn step() -> Result<(), crate::Error> {
        let mut ctrl = StepperCtrl::new(StepperConst::MOT_17HE15_1504S, PIN_DIR, PIN_STEP);
        ctrl.write_link(LinkedData::GEN); 
    
        ctrl.apply_inertia(Inertia(0.000_1));
    
        println!("Doing single step ... ");
        ctrl.step(Time(0.01))?;
        println!("Step done ... ");

        Ok(())
    }
    
    #[test]
    fn drive() -> Result<(), crate::Error> {
        let mut ctrl = StepperCtrl::new(StepperConst::MOT_17HE15_1504S, PIN_DIR, PIN_STEP);
        ctrl.write_link(LinkedData::GEN); 
    
        ctrl.apply_inertia(Inertia(0.4));
        ctrl.apply_force(Force(0.10));
    
        println!("Staring to move");
        ctrl.drive_rel(DELTA, OMEGA)?;
        println!("{} with max speed {:?}rad/s done", DELTA, OMEGA);

        Ok(())
    }
}