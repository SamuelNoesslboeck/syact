# Getting started

## Installation and Platforms

For installation, just type

```sh
cargo add stepper_lib
```

in your project command line, or add

```toml
stepper_lib = "0.11.4"
```

to your dependencies.Depending on which platform you are using, the build command for the library changes. It is not recommended to add a feature directly to the dependency, as the library will cause build errors if platforms are switched. 

Without any platform feature supplied, the library will automatically enter "simulation-mode", which means no pins will actually be written to and all inputs checked return `true`. I recommend creating platform features for different implementations:

```toml
[dependencies]
stepper_lib = "0.11.4"
# ...

[features]
rasp = [ "stepper_lib/rasp" ]
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
stepper_lib = "0.11.4"          # Importing the library

[features]
rasp = [ "stepper_lib/rasp" ]   # Creating the platform feature for our raspberry pi
```

Our stepper motor of the conveyor is connected a stepper motor controller, which itself is connected to a `Raspberry Pi`. Such connections usually have a logical signal for controlling the direction and another that sends a pulse for each step. In our programm, they are best defined as constants.

```rust
const PIN_DIR : u8 = 17;        // Pin of the directional signal
const PIN_STEP : u8 = 26;       // Pin of the step signal
```

Below are three approches listed to showcase some of the functionalities of the library. 

> **Note**
>
> If any of the types `Delta`, `Gamma`, `Omega`, ... are unclear to you, have a look at the [unit system](./unit_system.md)

### Using the predefined component

The easiest approch for this example would be using the predefined component from the library.

```rust 

```

### Direct appproach

Sometimes a rather direct way is easier to understand and simpler to maintain, just using a stepper motor struct we can solve this example using

```rust
// Define the radius of the powered conveyor roll as a constant with 5 millimeters
const R_ROLL : f32 = 5.0;

// Convert the linear conveyor speed to an angular speed of the motor
fn omega_for_motor(omega_conv : Omega) -> Omega {
    omega_conv / R_ROLL
}

fn direct_approach() {
    let mut ctrl = StepperCtrl::new(StepperConst::GEN, PIN_DIR, PIN_STEP);
    ctrl.write_link(LinkedData::GEN);
    ctrl.setup()?;
    
    // Apply a inertia to the conveyor (possible masses on the belt)
    ctrl.apply_inertia(Inertia(0.01));

    // Set the maximum speed of the conveyor tp 40 millimeters per second
    ctrl.set_omega_max(
        omega_for_motor(Omega(40.0))
    );

    println!("Driving forward with 0.5 speed");
    ctrl.drive(Direction::CW, 0.5)?;
    ctrl.await_inactive()?;

    println!(" -> Reached speed!");

    sleep(1.0);

    println!("Driving forward with 0.8 speed");
    ctrl.drive(Direction::CW, 0.8)?;
    ctrl.await_inactive()?;

    println!(" -> Reached speed!");

    sleep(2.0);

    println!("Driving backwards with 0.2 speed");
    ctrl.drive(Direction::CCW, 0.2)?;
    ctrl.await_inactive()?;

    println!("Reached speed!");

    sleep(1.0);

    println!("Finished!");
}
```


### Defining a custom component

In special cases the default structs offered by the library are not flexible enough, therefore a custom component is required. See [custom components](./components.md#custom-components). 