use core::f32::consts::PI;

use clap::{command, arg, value_parser};
// Include the library
use syact::prelude::*;

// Define distance and max speed defaults
const DELTA_DEF : Delta = Delta(30.0 * 2.0 * PI);       // Default value is 30 revolutions
const SPEED_F_DEF : f32 = 0.5;

fn main() -> Result<(), syact::Error> {
    // Parse cmd args
    let matches = command!() 
        .about("Moves a stepper motor with a generic PWM controller connected to the pins 'pin_step' and 'pin_dir' until either the measurement is done or
            the maximum distance `delta` is reached")
        .arg(arg!([pin_step] "Pin number of the step pin").value_parser(value_parser!(u8)))
        .arg(arg!([pin_dir] "Pin number of the direction pin").value_parser(value_parser!(u8)))
        .arg(arg!([pin_meas] "Pin number of the measurement pin").value_parser(value_parser!(u8)))
        .arg(arg!([dir] "Direction of the endswitch (`1` per default => `CW`)").value_parser(value_parser!(u8)))
        .arg(arg!([micro] "Enables microstepping with the given step value (1 per default)").value_parser(value_parser!(u8)))
        .arg(arg!([delta] "Delta (distance) of the movement in rad (2pi [1 rev] per default)").value_parser(value_parser!(f32)))
        .arg(arg!([speed_f] "Omega (velocity) of the movement in rad/s (10 rad/s per default)").value_parser(value_parser!(f32)))
        .get_matches();

    let pin_step : u8 = *matches.get_one("pin_step").expect("A valid step pin has to be provided");
    let pin_dir : u8 = *matches.get_one("pin_dir").expect("A valid direction pin has to be provided");
    let pin_meas : u8 = *matches.get_one("pin_meas").expect("A valid measurement pin has to be provided");
    let dir_u8 : u8 = *matches.get_one("dir").unwrap_or(&1);

    let micro_opt : Option<&u8> = matches.get_one("micro");
    let delta : Delta  = Delta(*matches.get_one("delta").unwrap_or(&DELTA_DEF.0));
    let speed_f : f32 = *matches.get_one("speed_f").unwrap_or(&SPEED_F_DEF);

    // Load data
    let inertia = std::env::var("INERTIA").ok().map(|v| v.parse::<Inertia>().unwrap()).unwrap_or(Inertia::ZERO);
    let force = std::env::var("FORCE").ok().map(|v| v.parse::<Force>().unwrap()).unwrap_or(Force::ZERO);
    let trigger = std::env::var("TRIGGER").ok().map(|v| v.parse::<bool>().unwrap()).unwrap_or(true);

    // Create the controls for a stepper motor
    let mut ctrl = Stepper::new(
        GenericPWM::new(pin_step, pin_dir)?, 
        StepperConst::MOT_17HE15_1504S
    );

    let data = SimpleMeasData { 
        set_gamma: Gamma::ZERO, 
        max_dist: delta, 
        meas_speed_f: speed_f, 

        add_samples: 2, 
        sample_dist: None
    };

    let mut switch = EndSwitch::new(pin_meas, trigger, Some(Direction::from_u8(dir_u8)));

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

    // Setup the switch
    switch.setup()?;

    // Append switch as interruptor
    ctrl.add_interruptor(Box::new(switch));

    // Starting the measurement
    println!("Starting measurement ... ");

    syact::meas::take_simple_meas(&mut ctrl, &data, 1.0)?;

    println!("Measurement done!");

    Ok(())
}  