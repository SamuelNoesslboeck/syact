use core::f32::consts::PI;

use clap::{command, arg, value_parser};
// Include the library
use syact::prelude::*;

// Define distance and max speed defaults
const DELTA_DEF : Delta = Delta(30.0 * 2.0 * PI);       // Default value is 30 revolutions

fn main() -> Result<(), syact::Error> {
    // Parse cmd args
    let matches = command!() 
        .about("Moves a stepper motor with a generic PWM controller connected to the pins 'pin_step' and 'pin_dir' until either the measurement is done or
            the maximum distance `delta` is reached")
        .arg(arg!([pin_step] "Pin number of the step pin").value_parser(value_parser!(u8)))
        .arg(arg!([pin_dir] "Pin number of the direction pin").value_parser(value_parser!(u8)))
        .arg(arg!([pin_meas] "Pin number of the measurement pin").value_parser(value_parser!(u8)))
        .arg(arg!([dir] "Direction of the endswitch (`1` => `CW`, `0` => `CCW`)").value_parser(value_parser!(u8)))
        .arg(arg!([micro] "Enables microstepping with the given step value (1 per default)").value_parser(value_parser!(u8)))
        .arg(arg!([delta] "Delta (distance) of the movement in rad (2pi [1 rev] per default)").value_parser(value_parser!(f32)))
        .arg(arg!([speed] "Speed factor of movement").value_parser(value_parser!(Factor)))
        .get_matches();

    let pin_step : u8 = *matches.get_one("pin_step").expect("A valid step pin has to be provided");
    let pin_dir : u8 = *matches.get_one("pin_dir").expect("A valid direction pin has to be provided");
    let pin_meas : u8 = *matches.get_one("pin_meas").expect("A valid measurement pin has to be provided");
    let dir_opt : Option<Direction> = matches.get_one("dir").map(|val| Direction::from_u8(*val));

    let micro_opt : Option<&MicroSteps> = matches.get_one("micro");
    let delta : Delta  = Delta(*matches.get_one("delta").unwrap_or(&DELTA_DEF.0));
    let speed : Factor = *matches.get_one("speed").unwrap_or(&Factor::from(0.5));

    // Load data
    let inertia = std::env::var("INERTIA").ok().map(|v| v.parse::<Inertia>().unwrap()).unwrap_or(Inertia::ZERO);
    let force = std::env::var("FORCE").ok().map(|v| v.parse::<Force>().unwrap()).unwrap_or(Force::ZERO);
    let trigger = std::env::var("TRIGGER").ok().map(|v| v.parse::<bool>().unwrap()).unwrap_or(false);
    let samples = std::env::var("SAMPLES").ok().map(|v| v.parse::<usize>().unwrap()).unwrap_or(2);

    // Create the controls for a stepper motor
    let mut device = Stepper::new(
        GenericPWM::new(pin_step, pin_dir)?, 
        StepperConst::MOT_17HE15_1504S
    );

    let data = SimpleMeasData { 
        set_gamma: Gamma::ZERO, 
        max_dist: delta, 
        meas_speed: speed, 

        add_samples: samples, 
        sample_dist: None
    };

    let mut switch = EndSwitch::new(trigger, dir_opt, UniInPin::new(pin_meas));

    // Link the component to a system
    device.set_config(StepperConfig { 
        voltage: 12.0,    // System voltage in volts
        overload_current: None
    }); 
    device.setup()?;

    if let Some(&micro) = micro_opt {
        device.set_microsteps(micro);
    }

    // Apply some loads
    device.apply_inertia(inertia);
    device.apply_gen_force(force)?;

    // Setup the switch
    switch.setup()?;

    // Append switch as interruptor
    device.add_interruptor(Box::new(switch));

    // Starting the measurement
    println!("Starting measurement ... ");

    let res = syact::meas::take_simple_meas(&mut device, &data, Factor::MAX)?;

    println!("Measurement done!\n");

    println!("{:?}", &res);
    println!("\nAdditional values:");
    println!(" - Gamma max: {}", res.gamma_max());
    println!(" - Gamma min: {}", res.gamma_min());
    println!(" - Max Inacc: {}", res.max_inacc());

    Ok(())
}  