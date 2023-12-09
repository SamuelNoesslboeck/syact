# Getting started

## Installation and Platforms

For installation, just type

```sh
cargo add syact
```

in your project command line, or add

```toml
syact = "0.12.0"
```

to your dependencies.Depending on which platform you are using, the build command for the library changes. It is not recommended to add a feature directly to the dependency, as the library will cause build errors if platforms are switched.

Without any platform feature supplied, the library will automatically enter "simulation-mode", which means no pins will actually be written to and all inputs checked return `true`. I recommend creating platform features for different implementations:

```toml
[dependencies]
syact = "0.12.0"
# ...

[features]
rasp = [ "syact/rasp" ]
# ... 
```

As the library is currently **only available for the raspberry pi**, the build command will look like this:

```sh
cargo build --features="rasp"
```

For more information, see [platforms](./platforms.md).

## Example

To demonstrate the tools of the library, let's assume we want to control a simple conveyor powered by a stepper motor. (See [the rust example](../examples/simple_conv/src/main.rs))

The cargo file of our project:

```toml
[package]
name = "simple_conv"
version = "0.1.0"
edition = "2021"

[dependencies]
syact = "0.12.0"          # Importing the library

[features]
rasp = [ "syact/rasp" ]   # Creating the platform feature for our raspberry pi
```

Our stepper motor of the conveyor is connected a stepper motor controller, which itself is connected to a `Raspberry Pi`. Such connections usually have a logical signal for controlling the direction and another that sends a pulse for each step. In our programm, they are best defined as constants.

```rust
const PIN_DIR : u8 = 17;        // Pin of the directional signal
const PIN_STEP : u8 = 26;       // Pin of the step signal
```

Below are three approches listed to showcase some of the functionalities of the library.

> **Note**
>
> - If any of the types `Delta`, `Gamma`, `Omega`, ... are unclear to you, have a look at the [unit system](./unit_system.md)
> - The motor used in this example ("MOT_17HE15_1504S") is predefined in the `StepperData` class. Please insert the data of the stepper motors you are using.

### Using the predefined component

The easiest approch for this example would be using the predefined component from the library.

```rust
// Define the radius of the powered conveyor roll as a constant with 5 millimeters
const R_ROLL : f32 = 10.0;

pub fn predefined() -> Result<(), syact::Error> {
    // First we crate our conveyor using a stepper motor and the radius of the roll that connects the belt to the motor
    let mut conv = Conveyor::new(
        Stepper::new(StepperConst::MOT_17HE15_1504S, PIN_DIR, PIN_STEP),        // The stepper motor
        R_ROLL
    );

    // Now we write the `StepperConfig` to our component. The `StepperConfig` is often data that is the same for 
    // all components, e.g. supply voltage
    conv.set_config(StepperConfig {
        u: 12.0,        // Voltage
        s_f: 1.5        // Safety factor, the higher the factor, the safer is the stepper to not jump over steps,
                        // however the performance suffers from very high safety factors
    });

    // Setup all the neccessary stuff for a stepper motor
    // => Spawns the thread to execute async movements
    conv.setup()?;
    
    // Apply a inertia to the conveyor (possible masses on the belt, 1.0kg estimated)
    conv.apply_inertia(Inertia(1.0));

    // Set the maximum speed of the conveyor to 200 millimeters per second
    conv.set_omega_max(Omega(200.0));

    println!("Driving forward with 0.5 speed");
    conv.drive(sylo::Direction::CW, 0.5)?;        // Drive with 100 mm/s speed (50%, 0.5)
    conv.await_inactive()?;

    println!(" -> Reached speed!");

    sleep(1.0);

    println!("Driving forward with 0.8 speed");
    conv.drive(sylo::Direction::CW, 0.8)?;        // Drive with 160 mm/s speed (80%, 0.8)
    conv.await_inactive()?;

    println!(" -> Reached speed!");

    sleep(2.0);

    println!("Driving backwards with 0.2 speed");
    conv.drive(sylo::Direction::CCW, 0.2)?;       // Drive with 40 mm/s speed in the opposite direction (20%, 0.2)
    conv.await_inactive()?;

    println!("Reached speed!");

    sleep(1.0);

    println!("Finished!");

    Ok(())
}
```

### Direct appproach

Sometimes a rather direct way is easier to understand and simpler to maintain, just using a stepper motor struct we can solve this example using

```rust
// Define the radius of the powered conveyor roll as a constant with 5 millimeters
const R_ROLL : f32 = 10.0;

// Convert the linear conveyor speed to an angular speed of the motor
fn omega_for_motor(omega_conv : Omega) -> Omega {
    omega_conv / R_ROLL
}

fn inertia_for_motor(inertia_conv : Inertia) -> Inertia {
    inertia_conv * R_ROLL * R_ROLL / 1_000_000.0      
    // R_ROLL has units millimeter, therefore a factor of 10^6 is required for conversion from kg to kgm^2
}

pub fn direct_approach() -> Result<(), syact::Error> {
    let mut device = Stepper::new(StepperConst::MOT_17HE15_1504S, PIN_DIR, PIN_STEP);
    device.set_config(StepperConfig::GEN);
    device.setup()?;
    
    // Apply a inertia to the conveyor (possible masses on the belt)
    device.apply_inertia(
        inertia_for_motor(Inertia(0.5))
    );

    // Set the maximum speed of the conveyor tp 40 millimeters per second
    device.set_omega_max(
        omega_for_motor(Omega(40.0))
    );

    println!("Driving forward with 0.5 speed");
    device.drive(sylo::Direction::CW, 0.5)?;
    device.await_inactive()?;

    println!(" -> Reached speed!");

    sleep(1.0);

    println!("Driving forward with 0.8 speed");
    device.drive(sylo::Direction::CW, 0.8)?;
    device.await_inactive()?;

    println!(" -> Reached speed!");

    sleep(2.0);

    println!("Driving backwards with 0.2 speed");
    device.drive(sylo::Direction::CCW, 0.2)?;
    device.await_inactive()?;

    println!("Reached speed!");

    sleep(1.0);

    println!("Finished!");

    Ok(())
}
```

### Defining a custom component

In special cases the default structs offered by the library are not flexible enough, therefore a custom component is required. See [custom components](./components.md#custom-components).
