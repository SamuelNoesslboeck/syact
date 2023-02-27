use core::time::Duration;
use std::thread;

use crate::{ctrl::servo::ServoDriver, data::ServoConst};

const PIN : u16 = 13;

#[test]
fn default_position() {
    let mut servo = ServoDriver::new(ServoConst::MG996R, PIN);
    servo.default_pos();

    println!("Moving servo ... ");

    thread::sleep(Duration::from_secs_f32(1.0));

    println!("Movement finished!")
}