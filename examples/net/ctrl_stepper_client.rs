use clap::{command, arg, value_parser};
use syiot::remote::ControlClient;

fn main() -> Result<(), syact::Error> {
    let matches = command!() 
        .about("Moves a stepper motor with a generic PWM controller connected to the pins 'pin_step' and 'pin_dir' by the given distance 
        'delta' with the maximum speed 'omega', optionally enabling microstepping with the microstepcount 'micro'")
        .arg(arg!([addr] "Network address to listen at").value_parser(value_parser!(String)))
        .arg(arg!([interface] "Local serial interface the magicbox is connected to").value_parser(value_parser!(String)))
        .get_matches();

    let addr : String = matches.get_one::<String>("addr").expect("A valid addr has to be provided!").clone();
    let interface : String = matches.get_one::<String>("interface").expect("A valid interface has to be provided!").clone();

    let mut mbox = magicbox::MagicBox::open(interface)?;

    let mut client = ControlClient::new();
    client.connect(syiot::remote::Transport::FramedTcp, addr)?;
    
    loop {
        client.send(mbox.update()?)?;
        std::thread::sleep(core::time::Duration::from_millis(10));
    }
}