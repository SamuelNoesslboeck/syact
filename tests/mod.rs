#[test]
fn __readme_test() {

use syact::prelude::*;

// Position of components
const POS : PositionMM = PositionMM(10.0);

// Create a new linear_axis (implements SyncActuator as their sub-component does)
let mut linear_axis = LinearAxis::new_belt_axis(
    // Some Demo-Actuator implementing (also implements SyncActuator)
    DemoActuator::new(), 
    Millimeters(0.5)    // The radius is set to 0.5, which means for each radian the motor moves, the linear_axis moves for 0.5 mm
);    

// Overwrite the current absolute position
linear_axis.overwrite_abs_pos(POS);

assert!((linear_axis.pos() - POS).abs() < Millimeters(0.001));      // Check with small tolerance



// Drive to 100mm with maximum speed
linear_axis.drive_abs(PositionMM(100.0), Factor::MAX).unwrap(); 

}