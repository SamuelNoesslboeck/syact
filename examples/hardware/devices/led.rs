use std::io::Write;

use clap::{command, arg, value_parser};
// Include the library
use syact::prelude::*;

fn prompt_enter() {
    let mut line = String::new();
    std::io::stdout().flush().unwrap();
    std::io::stdin().read_line(&mut line).expect("Error: Prompt enter");
}

fn main() -> Result<(), syact::Error> {
    // Parse cmd args
    let matches = command!() 
        .about("Controlls a LED at the given 'pin'")
        .arg(arg!([pin] "Pin number of the LED pin").value_parser(value_parser!(u8)))
        .get_matches();

    let pin : u8 = *matches.get_one("pin").expect("A valid pin has to be provided");

    // Create the pin for the LED
    let mut led = UniPin::new(pin)?.into_output();
    let mut state = false;

    led.set_low();
    
    println!("Created LED at pin: {}", pin);

    loop {
        println!("> Enter to toggle LED");
        prompt_enter();

        if state {
            state = false;
            led.set_low();
        } else {
            state = true;
            led.set_high();
        }
    }
}  