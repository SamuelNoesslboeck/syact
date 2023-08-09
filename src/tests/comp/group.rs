use crate as syact;

use syact::prelude::*;

#[derive(StepperCompGroup)]
struct SomeComps {
    base : Gear<Stepper>,
    arm1 : CylinderTriangle<Stepper>
}

#[test]
fn groups() -> Result<(), syact::Error> {
    let mut group = SomeComps {
        base: Gear::new(
            Stepper::new_gen(),
            0.2
        ),
        arm1: CylinderTriangle::new(
            Cylinder::new(
                Stepper::new_gen(),
                0.5
            ),
            100.0,
            150.0
        )
    }; 

    group.write_data(CompData::GEN);
    group.setup()
}