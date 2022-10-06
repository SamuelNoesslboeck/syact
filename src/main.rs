use std::vec;

use stepper_lib::{data::StepperData, math::{torque, start_frequency, acc_curve}};

fn main() {
    let mut data = StepperData::mot_17he15_1504s(12.0);
    data.j = 0.001;

    println!("{}Hz start frequency", start_frequency(&data));

    let mut curve = vec![];

    for i in 0 .. 201 {
        let omega : f64 = (i as f64) * 3.0;
        curve.push(torque(&data, omega));
    }

    dbg!(curve);
    // dbg!(acc_curve(&data, 1.0 / 1000.0, 1000));
}
