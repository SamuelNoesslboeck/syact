#![doc = r#"
Let us assume we want to control a simple stepper motor (in this example a 
[17HE15_1504_S](https://www.omc-stepperonline.com/index.php?route=product/product/get_file&file=2838/17HE15-1504S.pdf)) 
with a PWM controller connected to the BCM pins 27 and 19.

### Note 

The cargo.toml file specified below is when running the example on a raspberry pi

```toml
# ...

[dependencies]
# Include the library configured for the raspberry pi
syact = { version = "0.12.1", features = [ "rasp" ] } 

# ...
```
"#]

use core::f32::consts::PI;

use clap::{command, arg, value_parser};
// Include the library
use syact::prelude::*;

// Define distance and max speed defaults
const DELTA_DEF : Delta = Delta(2.0 * PI);
const OMEGA_DEF : Omega = Omega(20.0);

fn main() -> Result<(), syact::Error> {
    // Parse cmd args
    let matches = command!() 
        .about("Moves a stepper motor with a generic PWM controller connected to the pins 'pin_step' and 'pin_dir' by the given distance 
        'delta' with the maximum speed 'omega', optionally enabling microstepping with the microstepcount 'micro'")
        .arg(arg!([pin_step] "Pin number of the step pin").value_parser(value_parser!(u8)))
        .arg(arg!([pin_dir] "Pin number of the direction pin").value_parser(value_parser!(u8)))
        .arg(arg!([delta] "Delta (distance) of the movement in rad (2pi [1 rev] per default)").value_parser(value_parser!(f32)))
        .arg(arg!([omega] "Omega (velocity) of the movement in rad/s (10 rad/s per default)").value_parser(value_parser!(f32)))
        .arg(arg!([micro] "Enables microstepping with the given step value (1 per default)").value_parser(value_parser!(u8)))
        .get_matches();

    let pin_step : u8 = *matches.get_one("pin_step").expect("A valid step pin has to be provided");
    let pin_dir : u8 = *matches.get_one("pin_dir").expect("A valid direction pin has to be provided");

    let delta : Delta  = Delta(*matches.get_one("delta").unwrap_or(&DELTA_DEF.0));
    let omega : Omega = Omega(*matches.get_one("omega").unwrap_or(&OMEGA_DEF.0));
    let micro_opt : Option<&u8> = matches.get_one("micro");

    // Load data
    let inertia = std::env::var("INERTIA").ok().map(|v| v.parse::<Inertia>().unwrap()).unwrap_or(Inertia::ZERO);
    let force = std::env::var("FORCE").ok().map(|v| Force(v.parse::<f32>().unwrap())).unwrap_or(Force::ZERO);

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
    ctrl.apply_inertia(inertia);
    ctrl.apply_force(force);

    ctrl.set_omega_max(omega);

    print!("Staring to move ... ");
    ctrl.drive_rel(delta, 1.0)?;      // Move the motor
    println!("done!");
    println!("");

    println!("Movement data");
    println!("- Distance: {}rad", delta);
    println!("- Speed: {}rad", omega);

    Ok(())
}  