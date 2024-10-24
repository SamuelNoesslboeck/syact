use core::f32::consts::PI;

use syunit::prelude::*;
use syact::prelude::*;

pub fn prompt<T : core::str::FromStr + Copy>(msg : &str, default_opt : Option<T>) -> T {
    let mut input_string = String::new();

    loop {
        print!("{}", msg);
        std::io::Write::flush(&mut std::io::stdout()).unwrap();
        std::io::stdin().read_line(&mut input_string).unwrap();
    
        if let Ok(res) = T::from_str(input_string.trim()) {
            return res;
        } else {
            if let Some(default) = default_opt {
                return default;
            } else {
                println!("Bad input for type!");
            }
        }
    }
}

fn main() {
    // Print out header
    println!();
    println!("######################################");
    println!("#    Stepper Torque Curve Printer    #");
    println!("######################################");
    println!();
    println!("A helper programm to print out approximated stepper torque curves as they are used by the syact library.");
    println!("(c) Sy (Samuel Nösslböck) 2024  -  Version 1.0.1/2024/10/16");
    println!();
    println!("> NOTE: The torque curve of your motor can be very different to this approximation, compare with the torque curve given by the manufacturer and adjust parameters as required!");
    println!(); 

    // Get input data
    let consts = StepperConst {
        default_current: prompt("Current (Amp) per phase: ", None),
        inductance: prompt("Coil inductance (H): ", None),
        resistance: prompt("Coil resistance (Ohm): ", None),

        number_steps: prompt("Number of steps (1 - default: 200): ", Some(200)),
        torque_stall: prompt("Stall torque (Nm): ", None),
        inertia_motor: KgMeter2(1.0)  // Not required for calculations
    };

    let config = StepperConfig {
        voltage: prompt("Voltage (V - default: 12V): ", Some(12.0)),
        overload_current: None
    };

    // Print out data
    println!();
    dbg!(&consts);
    dbg!(&config);
    println!();

    println!(" Speed\t# Torque (Nm)"); 
    println!("##########################");

    // Print from 0 - 80 
    for i in 0 .. 5 {
        let rpm = i as f32 * 20.0;
        let torque = consts.torque_dyn(RadPerSecond(rpm / 60.0 * 2.0 * PI), &config);

        println!("  {}\t#  {}", rpm, torque);
    }

    // Print from 100 - 900
    for i in 1 .. 10 {
        let rpm = i as f32 * 100.0;
        let torque = consts.torque_dyn(RadPerSecond(rpm / 60.0 * 2.0 * PI), &config);

        println!("   {}\t#   {}", rpm, torque);
    }

    // Print 1000 & 2000
    for i in 1 .. 3 {
        let rpm = i as f32 * 1000.0;
        let torque = consts.torque_dyn(RadPerSecond(rpm / 60.0 * 2.0 * PI), &config);

        println!("   {}\t#   {}", rpm, torque);
    }
}