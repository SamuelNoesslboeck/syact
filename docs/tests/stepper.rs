#[cfg(feature = "std")]
mod single_motor 
{
    use core::f32::consts::PI;

    use crate::{Stepper, StepperConst, SyncComp};
    use crate::data::CompData;
    use crate::units::*;

    const PIN_DIR : u8 = 27;
    const PIN_STEP : u8 = 19;

    const DELTA : Delta = Delta(2.0 * PI);
    const OMEGA : Omega = Omega(10.0);

    #[test]
    fn step() -> Result<(), crate::Error> {
        let mut ctrl = Stepper::new(StepperConst::MOT_17HE15_1504S, PIN_DIR, PIN_STEP);
        ctrl.write_data(CompData { u: 12.0, s_f: 1.5 }); 
    
        ctrl.apply_inertia(Inertia(0.000_1));
    
        println!("Doing single step ... ");
        ctrl.step(Time(0.01))?;
        println!("Step done ... ");

        Ok(())
    }
    
    #[test]
    fn drive() -> Result<(), crate::Error> {
        let mut ctrl = Stepper::new(StepperConst::MOT_17HE15_1504S, PIN_DIR, PIN_STEP);
        ctrl.write_data(CompData::GEN); 
    
        ctrl.apply_inertia(Inertia(0.4));
        ctrl.apply_force(Force(0.10));
    
        println!("Staring to move");
        ctrl.drive_rel(DELTA, OMEGA)?;
        println!("{} with max speed {:?}rad/s done", DELTA, OMEGA);

        Ok(())
    }
}

#[cfg(feature = "std")]
mod curves {
    use crate::StepperConst;
    use crate::data::{CompVars, CompData};
    use crate::math;
    use crate::units::*;
   
    #[test]
    fn simple() {
        let data = StepperConst::GEN;
        let vars = CompVars { t_load: Force(0.1), j_load: Inertia(1.0), ..Default::default() };
        let data = CompData::GEN;

        let delta = Delta(0.62);
        let omega = Omega(10.0);

        dbg!(
            math::curve::create_simple_curve(&data, &vars, &data, delta, omega)
        );
    }
}