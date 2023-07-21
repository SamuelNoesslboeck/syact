use clap::{command, arg, value_parser};

use syact::prelude::*;
use syact::tool::Tongs; 

fn main() -> Result<(), syact::Error> {
    // Parse cmd args
    let matches = command!() 
        .about("Command for testing a tong driven by a servo motor using a pwm signal at the pin 'pin")
        .arg(arg!([pin_pwm] "Pin number of the pwm signal").value_parser(value_parser!(u8)))
        .get_matches();

    let pin_pwm : u8 = *matches.get_one("pin_pwm").expect("A pin for the PWM Signal has to be given!");

    // Create the tongs
    let mut tongs = Tongs::new(
        ServoDriver::new(ServoConst::MG996R, pin_pwm),
        0.7,
        0.4,
        50.0,
        Inertia(0.1)
    );

    loop {
        if tongs.closed() {
            println!("Open Tongs ... (Press Enter)");
        } else {
            println!("Close Tongs ... (Press Enter)");
        }

        let mut input_text = String::new();
        std::io::stdin()
            .read_line(&mut input_text)
            .expect("failed to read from stdin");
    
        tongs.toggle();
    }
}