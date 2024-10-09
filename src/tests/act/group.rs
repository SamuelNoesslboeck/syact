use crate::tests::SimPin;
use crate as syact;

use syact::prelude::*;

fn test<G : SyncActuatorGroup<T, 2>, T : SyncActuator + ?Sized + 'static>(_g : &G) {
    
}

#[derive(StepperActuatorGroup)]
struct SomeComps {
    base : Gear<Stepper<SimPin, SimPin>>,
    arm1 : LinearAxis<Stepper<SimPin, SimPin>>
}

#[test]
fn group_basics() -> Result<(), BuilderError> {
    let mut group = SomeComps {
        base: Gear::new(
            Stepper::new_gen()?,
            0.2
        ),
        arm1: LinearAxis::new(
            Stepper::new_gen()?,
            0.5
        )
    }; 
    test(&group);

    group.set_config(StepperConfig::GEN);
    Ok(())
}

// // Manual
// struct ManualGroup {
//     base : Gear<Stepper>,
//     arm1 : Gear<Stepper>
// }

// impl Setup for ManualGroup {
//     fn setup(&mut self) -> Result<(), syact::Error> {
//         self.base.setup()?;
//         self.arm1.setup()
//     }
// }

// impl<T : SyncActuator + 'static> SyncActuatorGroup<T, 2> for ManualGroup 
// where
//     Gear<Stepper> : AsRef<T>
// {
//     fn for_each<'a, F, R>(&'a self, mut func : F) -> [R; 2]
//         where 
//             F : FnMut(&'a T, usize) -> R 
//     {
//         let mut result : [R; 2] = unsafe { core::mem::zeroed() };

//         result[0] = func(self.base.as_ref(), 0);

//         result
//     }

//     fn for_each_mut<F, R>(&mut self, func : F) -> [R; 2]
//     where 
//         F : FnMut(&mut T, usize) -> R {
//         todo!()
//     }

//     fn try_for_each<'a, F, R, E>(&'a self, func : F) -> Result<[R; 2], E>
//     where 
//         F : FnMut(&'a T, usize) -> Result<R, E> {
//         todo!()
//     }

//     fn try_for_each_mut<F, R, E>(&mut self, func : F) -> Result<[R; 2], E>
//     where 
//         F : FnMut(&mut T, usize) -> Result<R, E> {
//         todo!()
//     }
// }

// pub fn test_manual<G : SyncActuatorGroup<T, 2>, T : SyncActuator + 'static>(_group_ref : &G) 
// where
//     Gear<Stepper> : AsRef<T>
// {
//     todo!()
// }

// #[test]
// fn group_manual() {
//     let group = ManualGroup {
//         base: Gear::new(
//             Stepper::new_gen(),
//             0.2
//         ),
//         arm1: Gear::new(
//             Stepper::new_gen(),
//             0.2
//         )
//     };

//     test_manual(&group);
// }