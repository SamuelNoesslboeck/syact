use std::io;

use crate::Time;
use crate::ctrl::pwm::PWMOutput;

const PIN_1 : u8 = 13;
const PIN_2 : u8 = 23;

#[test]
fn pwm_servo() {
    let mut pwm = PWMOutput::spawn(PIN_1);

    loop {
        println!("Input for Servo:// ");
        let mut input_text = String::new();
        io::stdin()
            .read_line(&mut input_text)
            .expect("failed to read from stdin");
    
        let trimmed = input_text.trim();
        match trimmed.parse::<f32>() {
            Ok(f) => pwm.set_period( Time(f / 1000.0), Time(0.02) ),
            Err(..) => println!("this was not a float: {}", trimmed),
        };
    }
}

#[test]
fn pwm_servo_2() {
    let mut pwm_1 = PWMOutput::spawn(PIN_1);
    let mut pwm_2 = PWMOutput::spawn(PIN_2);

    loop {
        println!("Input for Servo_1:// ");

        let mut input_1 = String::new();
        io::stdin()
            .read_line(&mut input_1)
            .expect("failed to read from stdin");
    
        let trimmed_1 = input_1.trim();
        match trimmed_1.parse::<f32>() {
            Ok(f) => pwm_1.set_period( Time(f / 1000.0), Time(0.02) ),
            Err(..) => println!("this was not a float: {}", trimmed_1),
        };

        println!("Input for Servo_2:// ");

        let mut input_2 = String::new();
        io::stdin()
            .read_line(&mut input_2)
            .expect("failed to read from stdin");
    
        let trimmed_2 = input_2.trim();
        match trimmed_2.parse::<f32>() {
            Ok(f) => pwm_2.set_period( Time(f / 1000.0), Time(0.02) ),
            Err(..) => println!("this was not a float: {}", trimmed_2),
        };
    }
}