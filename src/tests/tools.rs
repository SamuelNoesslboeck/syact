use crate::Gamma;
use crate::comp::tool::{Tongs, SimpleTool, AxialJoint, AxisTool};
use crate::ctrl::servo::ServoDriver;
use crate::data::ServoConst;

const TONGS_PIN : u8 = 23;
const AXIAL_JOINT_PIN : u8 = 13;

#[test]
fn tongs() {
    let mut tongs = Tongs::new(
        ServoDriver::new(ServoConst::MG996R, TONGS_PIN),
        0.7,
        0.4,
        50.0,
        0.1
    );

    loop {
        println!("Open/Close Tongs ... (Press Enter)");
        let mut input_text = String::new();
        std::io::stdin()
            .read_line(&mut input_text)
            .expect("failed to read from stdin");
    
        tongs.toggle();
    }
}

#[test] 
fn axial_joint() {
    let mut axial_joint = AxialJoint::new(
        ServoDriver::new(ServoConst::MG996R, AXIAL_JOINT_PIN),
        50.0,
        0.1
    );

    loop {
        println!("Angle for Bearing:// ");
        let mut input_text = String::new();
        std::io::stdin()
            .read_line(&mut input_text)
            .expect("failed to read from stdin");
    
        let trimmed = input_text.trim();
        match trimmed.parse::<f32>() {
            Ok(f) => axial_joint.rotate_abs(Gamma(f)),
            Err(..) => println!("this was not a float: {}", trimmed),
        };
    }
}