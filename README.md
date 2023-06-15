# stepper_lib

[![Rust]]([Rust-Workflow])
[![Crates.io version]][stepper_lib: crates.io]
[![stepper_lib: rustc 1.68+]][Rust 1.68]

[Rust]: https://github.com/SamuelNoesslboeck/stepper_lib/actions/workflows/rust.yml/badge.svg
[Rust-Workflow]: https://github.com/SamuelNoesslboeck/stepper_lib/actions/workflows/rust.yml
[Crates.io version]: https://img.shields.io/crates/v/stepper_lib.svg?style=flat-square
[stepper_lib: crates.io]: https://crates.io/crates/stepper_lib
[stepper_lib: rustc 1.68+]: https://img.shields.io/badge/stepper_lib-rustc_1.68+-lightgray.svg
[Rust 1.68]: https://blog.rust-lang.org/2023/03/09/Rust-1.68.0.html

> **Note** 
> 
> Many aspects of the library (for example the documentation) are not fully finished yet! 
> (Though I try to update it as frequent as possible)

A library for all types of components used in robots, including controlls for stepper motors, servo motors and more complex assemblies using said motors. Currently all implementations are made for the raspberry pi, though new implementations for more controllers are currently being made.

Basis library for the [sybot_lib]("https://github.com/SamuelNoesslboeck/sybot_lib)

- [Getting started](docs/getting_started.md)

### Goal

- Create an all-in-one library to control motors, read sensors and do basic calculations in rust.
- Keep it as easy to use as possible
- Specialize the library for hobbyists and tinkerers
- Offer options for static aswell as dynamic typings

## In Action

Let us assume we want to control a simple stepper motor (in this example a [17HE15_1504_S](https://www.omc-stepperonline.com/index.php?route=product/product/get_file&file=2838/17HE15-1504S.pdf)) with a PWM controller connected to the BCM pins 27 and 19.

<details>
<summary>
Click to show Cargo.toml
</summary>

```toml
# ...

[dependencies]
# Include the library configured for the raspberry pi
stepper_lib = { version = "0.11.6", features = [ "rasp" ] } 

# ...
```
</details>
<p></p>

```rust
use core::f32::consts::PI;

// Include the library
use stepper_lib::prelude::*;

// Pin declerations (BCM on raspberry pi)
const PIN_DIR : u8 = 27;
const PIN_STEP : u8 = 19;

// Define distance and max speed
const DELTA : Delta = Delta(2.0 * PI);
const OMEGA : Omega = Omega(10.0);

fn main() -> Result<(), stepper_lib::Error> {
    // Create the controls for a stepper motor
    let mut ctrl = Stepper::new(StepperConst::MOT_17HE15_1504S, PIN_DIR, PIN_STEP);
    // Link the component to a system
    ctrl.write_link(LinkedData { 
        u: 12.0,    // System voltage in volts
        s_f: 1.5    // System safety factor, should be at least 1.0
    }); 
    ctrl.setup()?;

    // Apply some loads
    ctrl.apply_inertia(Inertia(0.2));
    ctrl.apply_force(Force(0.10));

    ctrl.set_omega_max(OMEGA);

    println!("Staring to move");
    ctrl.drive_rel(DELTA, 1.0)?;      // Move the motor
    println!("Distance {}rad with max speed {:?}rad/s done", DELTA, OMEGA);

    Ok(())
}
```

(Source: [stepper_motor]("/examples/stepper_motor.rs"))

## Features

- [x] Motors
  - [x] [Stepper motors](/examples/stepper_motor.rs)
    - [x] Absolute/relative movements
    - [x] Continuous movements
    - [ ] Microstepping
    - [ ] Inverting logical signals
  - [x] [Servo motors](/docs/motors/servos.md)
  - [x] [DC motors](/docs/motors/dc_motors.md)
- [x] [Components](/docs/components.md)
  - [x] [Cylinder](/examples/cylinder.rs)
  - [x] Gear joint
  - [x] Cylinder-triangle
  - [x] [Conveyor](/examples/simple_conv/src/main.rs)
- [x] [Tools](/docs/tools.md)
  - [x] Tongs
  - [x] Axial joint
- [x] Calculation
  - [x] Complex acceleration curves
  - [x] Overloads
  - [x] Forces
  - [x] Inertias
- [ ] Measurements
  - [x] Simple switch
  - [ ] Rotary resolver
  - [x] Trait for custom measurements
- [ ] Extendable
  - [x] [Custom components](/docs/components.md#custom-components)
  - [x] Custom tools
  - [ ] Custom meaurement systems
- [x] [Unit system](/docs/unit_system.md)
- [ ] Minimal
  - [ ] Fully supports `no_std` environment
  - [ ] Available for basic embedded systems
- [ ] [Platforms](/docs/platforms.md)
  - [x] Raspberry Pi and similar
  - [ ] Arduino

## Issues and requests

If you encounter any issues or if you have any request for new features, feel free to create an issue at the [GitHub repo](https://github.com/SamuelNoesslboeck/stepper_lib).
