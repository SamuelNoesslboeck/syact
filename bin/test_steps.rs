use stepper_lib::{data::StepperData, ctrl::StepperCtrl};

fn main() {
    let args : Vec<String> = std::env::args().collect();

    let mut ctrl = StepperCtrl::new(
        StepperData::mot_17he15_1504s(12.0, 1.5), 
        3, 26);

    ctrl.apply_load_j(0.000_01);

    test_steps(&args, &mut ctrl);
}

fn test_steps(args : &Vec<String>, ctrl : &mut StepperCtrl)
{
    let mut steps = 200;
    let mut omega = 10.0;

    if args.len() > 1 {
        let arg2 = args[2].parse::<u64>();

        if arg2.is_ok() {
            steps = arg2.unwrap();
        }
    } 

    if args.len() > 2 {
        let arg3 = args[3].parse::<f32>();

        if arg3.is_ok() {
            omega = arg3.unwrap();
        }
    }

    println!("Starting test 'steps' ... ");
    ctrl.steps(steps, omega, stepper_lib::ctrl::UpdateFunc::None);
    println!("{} with max speed {}rad/s done", steps, omega);
}