use std::time::Instant;

use syact::prelude::*;
use syact::tests::SimulatedController;

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

pub fn prompt_opt<T : core::str::FromStr + Copy>(msg : &str) -> Option<T> {
    let mut input_string = String::new();

    print!("{}", msg);
    std::io::Write::flush(&mut std::io::stdout()).unwrap();
    std::io::stdin().read_line(&mut input_string).unwrap();

    T::from_str(input_string.trim()).ok()
}

fn main() {
    // Print out header
    println!();
    println!("############################");
    println!("#    Start-Stop Builder    #");
    println!("############################");
    println!();
    println!("A demonstration program for the `StartStopBuilder` of the syact library. ");
    println!("(c) Sy (Samuel Nösslböck) 2024  -  Version 1.0.0/2024/10/16");
    println!();
    println!("> NOTE: The torque curve of your motor can be very different to this approximation, compare with the torque curve given by the manufacturer and adjust parameters as required!");
    println!(); 

    // Simulated controller that does nothing for testing purposes
    let mut controller = SimulatedController::new();

    let mut builder = StartStopBuilder::new(StepperConst::MOT_17HE15_1504S, StepperConfig::VOLT12_NO_OVERLOAD).unwrap();

    println!("Stepper constants used: ");
    println!("{:?}", builder.consts());
    println!();

    let distance : RelDist = prompt("Please enter the distance to travel (radians - default: 2.0): ", Some(RelDist(2.0)));
    
    builder.set_velocity_max(prompt_opt("Max velocity (optional): ")).unwrap();
    builder.set_acceleration_max(prompt_opt("Max acceleration (optional): ")).unwrap();


    builder.set_drive_mode(DriveMode::FixedDistance(distance, Velocity::ZERO, Factor::MAX), &mut controller).unwrap();

    let mut all_node = Time::ZERO;
    let mut steps = 0;

    let inst = Instant::now();

    for (i, node) in builder.enumerate() {
        steps = i + 1;
        all_node = node;
    }

    let calc_time = inst.elapsed().as_secs_f32();

    if steps > 0 {
        println!("# Movement summary");
        println!("| > Steps: {}", steps);
        println!("| > Time per step: {}s", all_node);
        println!("| > Calculation time: {}s", calc_time);
    } else {
        println!("The stepper has not moved a single node!");
    }
}