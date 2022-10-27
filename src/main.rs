use std::{time, thread};
use stepper_lib::{data::StepperData, controller::{PwmStepperCtrl, StepperCtrl}};

fn main() {
    let args : Vec<String> = std::env::args().collect();

    if args.len() == 1 {
        println!("No test name given!");
        return;
    }

    let mut ctrl = PwmStepperCtrl::new(
        StepperData::mot_17he15_1504s(12.0), 
        3, 26);

    // ctrl.sf = 3.0;
    ctrl.data.t_s /= 2.0;
    ctrl.data.j = 0.001;

    match args[1].as_str() {
        "step" => test_step(&args, &mut ctrl),
        "steps" => test_steps(&args, &mut ctrl), 

        _ => println!("No test with the given name found")
    };
}

fn test_step(_args : &Vec<String>, ctrl : &mut PwmStepperCtrl)
{
    println!("Starting test 'step' ... ");
    ctrl.step(0.01);
    println!("Step done ... ");
}

fn test_steps(args : &Vec<String>, ctrl : &mut PwmStepperCtrl)
{
    let mut steps = 200;
    let mut omega = 10.0;

    if args.len() > 2 {
        let arg2 = args[2].parse::<u64>();

        if arg2.is_ok() {
            steps = arg2.unwrap();
        }
    } 

    if args.len() > 3 {
        let arg3 = args[3].parse::<f64>();

        if arg3.is_ok() {
            omega = arg3.unwrap();
        }
    }

    println!("Starting test 'steps' ... ");
    ctrl.steps(steps, omega);
    println!("{} with max speed {}rad/s done", steps, omega);
}

fn test_repeat(args : &Vec<String>, ctrl : &mut PwmStepperCtrl)
{
    let mut steps = 100;
    let mut omega = 5.0;
    let mut times = 50;

    if args.len() > 2 {
        let arg2 = args[2].parse::<u64>();

        if arg2.is_ok() {
            steps = arg2.unwrap();
        }
    } 

    if args.len() > 3 {
        let arg3 = args[3].parse::<f64>();

        if arg3.is_ok() {
            omega = arg3.unwrap();
        }
    }

    for i in 0 .. times {
        
    }
}