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

    match args[1].as_str() {
        "step" => test_step(&mut ctrl),
        "steps" => test_steps(&mut ctrl), 
        _ => println!("No test with the given name found")
    };
}

fn test_step(ctrl : &mut PwmStepperCtrl)
{
    println!("Starting test 'step' ... ");
    ctrl.step();
    println!("Step done ... ");
}

fn test_steps(ctrl : &mut PwmStepperCtrl)
{
    println!("Starting test 'steps' ... ");
    ctrl.steps(50, 5.0);
    println!("50 steps done");
}