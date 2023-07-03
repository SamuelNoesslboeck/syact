#![doc = "
This example controls a simple pair of tongs using a servo motor.

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

// Include the library
use syact::prelude::*;

// Pin declerations (BCM on raspberry pi)
const PIN_PWM : u8 = 27;

fn main() -> Result<(), syact::Error> {
    // Create the tongs controller
    let mut tongs = tool::Tongs::new(
        ServoDriver::new(ServoConst::MG996R, PIN_PWM),    // The tongs use a MG996R servo connected to the BCM pin 27   
        0.2,        // when the pwm signal is at 20%, the tongs are open
        0.8,        // when the pwm signal is at 80%, the tongs are closed
        100.0,      // the tongs are 100 millimeters long
        Inertia(0.2)        // the tongs have a weight of 0.2 kg
    );

    tongs.mount();

    tongs.activate();
    sleep(Duration::from_secs(1));
    tongs.deactivate();

    Ok(())
}