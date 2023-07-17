#![doc = "
Let us assume we want to control a simple stepper motor (in this example a 
[17HE15_1504_S](https://www.omc-stepperonline.com/index.php?route=product/product/get_file&file=2838/17HE15-1504S.pdf)) 
with a PWM controller connected to the BCM pins 27 and 19.

### Note 

The cargo.toml file specified below is when running the example on a raspberry pi

```toml
# ...

[dependencies]
# Include the library configured for the raspberry pi
syact = { version = \"0.11.6\", features = [ \"rasp\" ] } 

# ...
```
"]

use core::f32::consts::PI;

// Include the library
use syact::prelude::*;

// Pin declerations (BCM on raspberry pi)
const PIN_DIR : u8 = 27;
const PIN_STEP : u8 = 19;

// Define distance and max speed
const DELTA : Delta = Delta(2.0 * PI);
const OMEGA : Omega = Omega(10.0);

fn main() -> Result<(), syact::Error> {
    // Create the controls for a stepper motor
    let mut ctrl = Stepper::new(StepperConst::MOT_17HE15_1504S, PIN_DIR, PIN_STEP);
    // Link the component to a system
    ctrl.write_data(CompData { 
        u: 12.0,    // System voltage in volts
        s_f: 1.5    // System safety factor, should be at least 1.0
    }); 
    ctrl.setup()?;

    // Apply some loads
    ctrl.apply_inertia(Inertia(0.2));
    ctrl.apply_force(Force(0.10));

    ctrl.set_omega_max(OMEGA);

    println!("Staring to move");
    ctrl.drive_rel(DELTA, 1.0)?;      // Move the motor
    println!("Distance {}rad with max speed {:?}rad/s done", DELTA, OMEGA);

    Ok(())
}