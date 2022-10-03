use stepper_lib::{data::StepperData, math::{torque, start_frequency, acc_curve}};

fn main() 
{
    let mut data = StepperData::mot_17he15_1504s(12.0);
    data.j = 0.001;

    // println!("{}Hz start frequency", start_frequency(&data));

    // for i in 0 .. 21 {
    //     let f : f64 = i as f64 * 250.0;
    //     println!("{} Hz\t{}Nm", f, torque(&data, f));
    // }

    dbg!(acc_curve(&data, 1.0 / 5000.0, 1000));
}
