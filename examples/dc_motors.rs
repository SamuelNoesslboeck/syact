#![doc = "
In this example a simple DC-Motor is controlled.

### Note 

The cargo.toml file specified below is when running the example on a raspberry pi

```toml
# ...

[dependencies]
# Include the library configured for the raspberry pi
syact = { version = \"0.11\", features = [ \"rasp\" ] } 

# ...
```
"]

use core::time::Duration;
use std::thread::sleep;

use syact::prelude::*;

// Pin declerations
/// Pin for PWM-Signal in clockwise direction
const PIN_CW : u8 = 27;
/// Pin for PWM-Signal in counter-clockwise direction
const PIN_CCW : u8 = 19; 

/// Defines the frequency of the PWM Signal
const FREQ : Omega = Omega(100.0);

fn main() -> Result<(), syact::Error> {
    // Create the motor with the constants above
    let mut motor = DcMotor::new(
        SoftwarePWM::new(PIN_CW)
            .with_freq(FREQ),
        SoftwarePWM::new(PIN_CCW)
            .with_freq(FREQ)
    );
    
    // Setup the PWM signals
    motor.setup()?; 

    // Drive clockwise for 2 seconds with max speed
    println!("Driving CW, max speed ... ");
    motor.drive(sylo::Direction::CW, 1.0);
    sleep(Duration::from_secs(2));

    // Drive coutner-clockwise for 1 second with half speed
    println!("Driving CCW, half speed ... ");
    motor.drive(sylo::Direction::CCW, 0.5);
    sleep(Duration::from_secs(1));

    // Drive counter-clockwise for 3 seconds with max speed
    println!("Driving CCW, full speed ... ");
    motor.drive(sylo::Direction::CCW, 1.0);
    sleep(Duration::from_secs(3));

    Ok(())
}