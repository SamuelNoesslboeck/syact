# syact

A library that defines actuators and their interaction with each other.

This library is intended to define bindings that can then be implemented by device-specific library, making their components available for use by general libraries like [sybot](https://github.com/SamuelNoesslboeck/sybot).

## In Action

```rust, ignore

use syact::intr::{Interruptor, InterruptReason};
use syact::{Interruptible, prelude::*};

// The following code features units for the syunits crate: <https://github.com/SamuelNoesslboeck/syunits> 
// Which is also embedded into syact::units

// Create a new linear_axis (implements [SyncActuator] as their sub-component does)
let mut linear_axis = LinearAxis::new_belt_axis(
    // Some Demo-Actuator implementing [SyncActuator]
    DemoActuator::new(), 
    // The ratio is set to 0.5, which means for each radian the motor moves, the linear_axis moves for 0.5 mm
    // This could be e.g. the radius of a pulley or gear for a belt drive
    Millimeters(0.5)    
);    

/* Position */
    // Overwrite the current absolute position
    linear_axis.overwrite_position(PositionMM(10.0));

    // Check with small tolerance
    assert!((linear_axis.position() - PositionMM(10.0)).abs() < Millimeters(0.001));      

    // Check the childs (DemoActuator) position
    // As for each radian, the axis moves 0.5mm, the DemoActuator should have (1 / 0.5 = 2) twice the distance
    assert!((linear_axis.child().position() - PositionRad(20.0)).abs() < Radians(0.001));
/**/

/* General functionalities */
    // SyncActuator offers a few default functions that are mostly automatically implemented and converted, no by-hand calculations required

    linear_axis.set_velocity_max(Some(MMPerSecond(100.0))).unwrap();
    linear_axis.set_velocity_max(None).unwrap();         // Limits can be reset using `None`

    linear_axis.set_acceleration_max(Some(MMPerSecond2(1000.0))).unwrap();
    linear_axis.set_jolt_max(Some(MMPerSecond3(10000.0))).unwrap();

    // (Note that for the testing purposes the DemoActuator performs all movements instantly, regardless of the limits set above, they are
    // just showcasing standard SyncActuator functions)
/**/

/* Interruptors & Measurements */
    // A simple demo interruptor that will stop the component as soon as its position is over a certain value
    // In real code this could be e.g. an endstop switch, see [syact::meas::EndStop]
    pub struct DemoInterruptor {
        pub at_pos : PositionRad
    }

    impl DemoInterruptor {
        // This function
        fn create_interruptor(self) -> Interruptor<Rotary> {
            Interruptor::new(
                Box::new(move |vars| {
                    if vars.position > self.at_pos {
                        Ok(Some(InterruptReason::HardwareEndReached))
                    } else {
                        Ok(None)
                    }
                }), 
                None        // This interruptor is not directionally dependent
            )
        }
    }

    // For the demo actuator, the interrupt handler is exposed, how one implements this does not really matter, may it be an accessor etc.
    linear_axis.actuator.intr_handler.add_intr(
        (DemoInterruptor { 
            at_pos: PositionRad(100.0)      // Means 50mm for the axis
        }).create_interruptor()
    );

    // Drive to 100mm with maximum speed
    linear_axis.drive_abs(PositionMM(100.0), Factor::MAX).unwrap(); 

    // Check if it has stopped because of the interruptor
    assert_eq!(linear_axis.take_intr_reason(), Some(InterruptReason::HardwareEndReached));

    // Check if it has stopped at the right position with some more tolerance 
    // (for most real actuators this tolerance is much smaller, for example 1.8deg for most stepper motors)
    assert!((linear_axis.child().position() - PositionRad(100.0)).abs() < Radians(0.2));
/**/

/* Calibration & Routines */
    // Our custom interruptor can now be used to call a standard calibration routine that the crate offers
    let params = syact::rout::SimpleRefPosParams::new( 
        PositionMM(0.0),        // The position it will be updated to after calibration
        Millimeters(100.0),         // The distance that will be moved before throwing an error
        Factor::HALF         // Speed at which the measurement will be conducted
    );

    // Start the routine
    syact::rout::find_ref_pos_simple(&mut linear_axis, &params, Factor::MAX).unwrap();

    // Check if the position has been updated correctly
    assert!((linear_axis.position() - PositionMM(0.0)).abs() < Millimeters(0.001));   
/**/
```

## Further libraries

- **Basic libraries**
  - [syunit](https://github.com/SamuelNoesslboeck/syunit): Flexible unit system and basis for many traits in this library, furthermore embedded into it in the `units` module
  - [sykin](https://github.com/SamuelNoesslboeck/sykin): Basic kinematic equations
- **Advanced libraries**
  - [sybot](https://github.com/SamuelNoesslboeck/sybot): Follow up library that defines robots out of traits in this library
- **Device-specific libraries**
  - [systep](https://github.com/SamuelNoesslboeck/systep): Stepper motors and drivers library, implementing `embedded-hal`

## Issues and requests

If you encounter any issues or if you have any request for new features, feel free to create an issue at the [GitHub repo](https://github.com/SamuelNoesslboeck/syact).
