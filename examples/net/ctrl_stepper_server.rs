use clap::{command, arg, value_parser, crate_authors, crate_version};
use syact::prelude::*;
use syiot::remote::ControlHandler;

pub struct Handler<'a> {
    stepper : &'a mut Stepper
}

impl<'a> ControlHandler<magicbox::State> for Handler<'a> {
    fn on_accept(&mut self) {
        println!(" => Controller accepted")
    }

    fn on_msg(&mut self, msg : Result<magicbox::State, syiot::Error>) {
        if let Ok(state) = msg {
            let dir = if state.joystick_x >= 0 {
                Direction::CW
            } else {
                Direction::CCW
            };

            self.stepper.drive(dir, (state.joystick_x.abs() as f32 / 100.0).min(1.0)).unwrap();
        } else {
            dbg!(msg.err());
        }
    }

    fn on_disconnect(&mut self) {
        println!(" => Controller disconnected!");
    }
}

fn main() -> Result<(), syact::Error> {
    let matches = command!()
        .author(crate_authors!())
        .version(crate_version!())
        .about("Moves a stepper motor with a generic PWM controller connected to the pins 'pin_step' and 'pin_dir' by the given distance 
        'delta' with the maximum speed 'omega', optionally enabling microstepping with the microstepcount 'micro'")
        .arg(arg!([pin_step] "Pin number of the step pin").value_parser(value_parser!(u8)))
        .arg(arg!([pin_dir] "Pin number of the direction pin").value_parser(value_parser!(u8)))
        .arg(arg!([addr] "Network address to listen at").value_parser(value_parser!(String)))
        .get_matches();

    let pin_step : u8 = *matches.get_one("pin_step").expect("A valid step pin has to be provided");
    let pin_dir : u8 = *matches.get_one("pin_dir").expect("A valid direction pin has to be provided");

    let addr : String = matches.get_one::<String>("addr").expect("A valid addr has to be provided!").clone();

    let mut stepper = Stepper::new(GenericPWM::new(pin_step, pin_dir)?, StepperConst::GEN);
    stepper.set_config(StepperConfig::GEN);
    stepper.setup()?; 

    let device = syiot::remote::Control::new(Handler { stepper: &mut stepper });

    device.listen(syiot::remote::Transport::FramedTcp, addr)?;
    device.run();

    Ok(())
}