# Tools

The library also includes various simple tools like tongs, axial joints and so on. These tools are controlled by a servo controller, though other types of motors can be used in custom tools by implementing the `Tool` trait, just like `SyncComp`.

This example controls a simple pair of tongs using a servo motor.

```rust
use core::time::Duration;

use std::thread::sleep;

use syact::{SimpleTool, Tool};
use syact::act::tool::Tongs;
use syact::device::servo::ServoDriver;
use syact::data::servo::ServoConst;

// Include the unit system
use syact::units::*;

// Pin declerations (BCM on raspberry pi)
const PIN_PWM : u8 = 27;

fn main() -> Result<(), syact::Error> {
    // Create the tongs controller
    let mut tongs = Tongs::new(
        ServoDriver::new(ServoConst::MG996R, PIN_PWM),    // The tongs use a MG996R servo connected to the BCM pin 27   
        0.2,        // when the pwm signal is at 20%, the tongs are open
        0.8,        // when the pwm signal is at 80%, the tongs are closed
        100.0,      // the tongs are 100 millimeters long
        Inertia(0.2)        // the tongs have a weight of 0.2 kg
    );

    tongs.setup()?;

    tongs.activate();
    sleep(Duration::from_secs(1));
    tongs.deactivate();

    Ok(())
}
```
