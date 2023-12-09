use crate as syact;

use syact::prelude::*;

fn test<G : SyncActuatorGroup<T, 2>, T : SyncActuator + ?Sized + 'static>(_g : &G) {
    
}

#[derive(StepperActuatorGroup)]
struct SomeComps {
    base : Gear<Stepper>,
    arm1 : LinearAxis<Stepper>
}

#[test]
fn groups() -> Result<(), syact::Error> {
    let mut group = SomeComps {
        base: Gear::new(
            Stepper::new_gen(),
            0.2
        ),
        arm1: LinearAxis::new(
            Stepper::new_gen(),
            0.5
        )
    }; 
    test(&group);

    group.set_config(StepperConfig::GEN);
    group.setup()
}