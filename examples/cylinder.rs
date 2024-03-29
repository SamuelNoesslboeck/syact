#![doc = "
In this example we drive a cylinder by a certain amount of millimeters.

### Note 

The cargo.toml file specified below is when running the example on a raspberry pi

```toml
# ...

[dependencies]
# Include the library configured for the raspberry pi
syact = { version = \"0.12.1\", features = [ \"rasp\" ] } 

# ...
```
"]

// Include the library
use syact::prelude::*;

// Pin declerations (BCM on raspberry pi)
const PIN_DIR : u8 = 27;
const PIN_STEP : u8 = 19;

// Define distance and max speed
const DELTA : Delta = Delta(10.0);      // 10 millimeters
const OMEGA : Omega = Omega(20.0);      // 20 millimeters per second

fn main() -> Result<(), syact::Error> {
    // Create the controls for a stepper motor
    let mut cylinder = Cylinder::new(
        Stepper::new(GenericPWM::new(PIN_STEP, PIN_DIR)?, StepperConst::MOT_17HE15_1504S),
        1.273       // Spindle pitch of the cylinder, per radian the cylinder extends for 1.273 millimeters,
                    // this factor calculates out of the pitch per revolve (8mm) divided by 2*PI (for radians) 
    );
    // Link the component to a system
    cylinder.write_data(CompData { 
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
