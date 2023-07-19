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
syact = { version = \"0.12.1\", features = [ \"rasp\" ] } 

# ...
```
"]

use core::f32::consts::PI;

use clap::{command, arg, value_parser};
// Include the library
use syact::prelude::*;

// Define distance and max speed
const DELTA : Delta = Delta(2.0 * PI);
const OMEGA : Omega = Omega(10.0);

fn main() -> Result<(), syact::Error> {
    // Parse cmd args
    let matches = command!() // requires `cargo` feature
        .arg(arg!([pin_step] "Pin number of the step pin").value_parser(value_parser!(u8)))
        .arg(arg!([pin_dir] "Pin number of the direction pin").value_parser(value_parser!(u8)))
        .arg(arg!([speed_f] "Speed factor to use for the movement").value_parser(value_parser!(f32)))
        .arg(arg!([micro] "Enables microstepping with the given step value").value_parser(value_parser!(u8))).get_matches();

    let pin_step : u8 = *matches.get_one("pin_step").expect("A valid step pin has to be provided");
    let pin_dir : u8 = *matches.get_one("pin_dir").expect("A valid direction pin has to be provided");

    let speed_f : f32  = *matches.get_one("speed_f").unwrap_or(&1.0);
    let micro_opt : Option<&u8> = matches.get_one("micro");

    // Create the controls for a stepper motor
    let mut ctrl = Stepper::new(
        GenericPWM::new(pin_step, pin_dir)?, 
        StepperConst::MOT_17HE15_1504S
    );

    // Link the component to a system
    ctrl.write_data(CompData { 
        u: 12.0,    // System voltage in volts
        s_f: 1.5    // System safety factor, should be at least 1.0
    }); 
    ctrl.setup()?;

    if let Some(&micro) = micro_opt {
        ctrl.set_micro(micro);
    }

    // Apply some loads
    ctrl.apply_inertia(Inertia(0.01));
    ctrl.apply_force(Force(0.10));

    ctrl.set_omega_max(OMEGA);

    println!("Staring to move");
    ctrl.drive_rel(DELTA, speed_f)?;      // Move the motor
    println!("Distance {}rad with max speed {:?}rad/s done", DELTA, OMEGA);

    Ok(())
}