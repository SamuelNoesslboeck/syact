#![doc = "
In this example we drive a cylinder by a certain amount of millimeters.

### Note 

The cargo.toml file specified below is when running the example on a raspberry pi

```toml
# ...

[dependencies]
# Include the library configured for the raspberry pi
stepper_lib = { version = \"0.11\", features = [ \"rasp\" ] } 

# ...
```
"]

// Include components and data
use stepper_lib::{StepperCtrl, StepperConst, SyncComp, Setup};
use stepper_lib::comp::Cylinder;
use stepper_lib::data::LinkedData;
// Include the unit system
use stepper_lib::units::*;

// Pin declerations (BCM on raspberry pi)
const PIN_DIR : u8 = 27;
const PIN_STEP : u8 = 19;

// Define distance and max speed
const DELTA : Delta = Delta(10.0);      // 10 millimeters
const OMEGA : Omega = Omega(20.0);      // 20 millimeters per second

fn main() -> Result<(), stepper_lib::Error> {
    // Create the controls for a stepper motor
    let mut cylinder = Cylinder::new(
        StepperCtrl::new(StepperConst::MOT_17HE15_1504S, PIN_DIR, PIN_STEP),
        1.273       // Spindle pitch of the cylinder, per radian the cylinder extends for 1.273 millimeters,
                    // this factor calculates out of the pitch per revolve (8mm) divided by 2*PI (for radians) 
    );
    // Link the component to a system
    cylinder.write_link(LinkedData { 
        u: 12.0,    // System voltage in volts
        s_f: 1.5    // System safety factor, should be at least 1.0
    }); 
    cylinder.setup()?;

    // Apply some loads
    cylinder.apply_inertia(Inertia(0.2));
    cylinder.apply_force(Force(0.10));

    println!("Staring to move ... ");
    let delta_real = cylinder.drive_rel(DELTA, 1.0)?;         // Move the cylinder
    println!("Distance {}mm with max speed {:?}mm/s done", delta_real, OMEGA);

    Ok(())
}
