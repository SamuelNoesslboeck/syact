use std::{thread, time};
use std::io::{Write, stdout};

use crossterm::{QueueableCommand, cursor, terminal, ExecutableCommand};
use syact::prelude::*;

/// Plots the gamma and omega values to the console during an async movment process
fn plot(stepper : &mut Stepper) {
    let mut stdout = stdout();

    stdout.execute(cursor::Hide).unwrap();
    while stepper.is_active() {
        stdout.queue(cursor::SavePosition).unwrap();
        stdout.write_all(format!("Gamma: {}, Speed: {}", stepper.gamma(), stepper.omega_cur()).as_bytes()).unwrap();
        stdout.queue(cursor::RestorePosition).unwrap();
        stdout.flush().unwrap();
        thread::sleep(time::Duration::from_millis(100));

        stdout.queue(cursor::RestorePosition).unwrap();
        stdout.queue(terminal::Clear(terminal::ClearType::FromCursorDown)).unwrap();
    }
    stdout.execute(cursor::Show).unwrap();
}

fn main() -> Result<(), syact::Error> {
    // Initialize stepper
    let mut stepper = Stepper::new_gen();
    stepper.set_config(StepperConfig::GEN);
    stepper.setup()?;

    // Apply loads
    // stepper.apply_inertia(Inertia(0.2));

    // First drive
        stepper.drive_rel_async(Delta(30.0), 1.0)?;
        plot(&mut stepper);

        println!("First movement done! ( Delta: {}, Gamma: {}, Omega_cur: {})", stepper.await_inactive()?, stepper.gamma(), stepper.omega_cur());
    // 

    // Continous drive in one direction
        stepper.drive(Direction::CW, 0.2)?;

        plot(&mut stepper);
    // 

    Ok(())    
}