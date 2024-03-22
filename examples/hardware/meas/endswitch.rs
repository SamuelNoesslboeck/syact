use std::{thread, time};
use std::io::{Write, stdout};

use clap::{command, arg, value_parser};
use crossterm::event::{Event, KeyCode, KeyEvent, KeyEventKind, KeyEventState, KeyModifiers};
use crossterm::{QueueableCommand, cursor, event, terminal, ExecutableCommand};
use syact::device::led::LED;
use syact::prelude::*;

/// Plots the gamma and omega values to the console during an async movment process
fn plot(switch : &mut EndSwitch, mut led_opt : Option<LED>) -> Result<(), syact::Error> {
    let mut stdout = stdout();

    stdout.execute(cursor::Hide).unwrap();
    loop {
        let state = unsafe { switch.meas().unwrap_unchecked() };

        // Plot
        stdout.queue(cursor::SavePosition).unwrap();    
        stdout.write_all(format!("   |- Switch state: {}", state).as_bytes())?;
        stdout.queue(cursor::RestorePosition).unwrap();
        stdout.flush().unwrap();

        if let Some(led) = &mut led_opt {
            led.set(state);
        }

        thread::sleep(time::Duration::from_millis(20));

        stdout.queue(cursor::RestorePosition).unwrap();
        stdout.queue(terminal::Clear(terminal::ClearType::FromCursorDown)).unwrap();

        match event::read().unwrap() {
            Event::Key(KeyEvent {
                code: KeyCode::Char('q'),
                modifiers: KeyModifiers::NONE,
                kind: KeyEventKind::Press,
                state: KeyEventState::NONE
            }) => {
                break;
            },
            _ => ()
        };
    }

    stdout.execute(cursor::Show).unwrap();
    println!("   | > Test ended");

    Ok(())
}

fn main() -> Result<(), syact::Error> {
    // Parse cmd args
    let matches = command!() 
        .about("Plots the value of an endswitch at the given 'pin' and additionally writes it out to an 'led'")
        .arg(arg!([pin] "Pin number of the endswitch").value_parser(value_parser!(u8)))
        .arg(arg!([led] "Pin number of the LED pin to plot the value to").value_parser(value_parser!(u8)))
        .get_matches();

    let pin : u8 = *matches.get_one("pin").expect("A valid pin has to be provided");
    let led_pin : Option<&u8> = matches.get_one("led");

    let mut switch = EndSwitch::new(true, None, UniInPin::new(pin));
    switch.setup()?;

    let led = led_pin.map(|lpin| {
        let mut led = LED::new(*lpin);
        led.setup().unwrap();
        led
    });

    println!("  Testing endswitch  ");
    println!("=====================");

    if let Ok(v) = std::env::var("SYACT_DEVICE_NAME") {
        println!(" |- Device name: {}", v);
    }

    println!(" |- Pin-Number: {}", pin);
    println!(" |");
    println!(" | => Press Q to quit!  ");
    println!("   |");

    plot(&mut switch, led)
}