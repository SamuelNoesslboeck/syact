# syact

[![Rust]]([rust-workflow])
[![Crates.io version]][syact: crates.io]
[![syact: rustc 1.68+]][Rust 1.68]

[Rust]: https://github.com/SamuelNoesslboeck/syact/actions/workflows/rust.yml/badge.svg
[rust-workflow]: https://github.com/SamuelNoesslboeck/syact/actions/workflows/rust.yml
[Crates.io version]: https://img.shields.io/crates/v/syact.svg?style=flat-square
[syact: crates.io]: https://crates.io/crates/syact
[syact: rustc 1.68+]: https://img.shields.io/badge/syact-rustc_1.68+-lightgray.svg
[Rust 1.68]: https://blog.rust-lang.org/2023/03/09/Rust-1.68.0.html

> **Note**
>
> Many aspects of the library (for example the documentation) are not fully finished yet!
> (Though I try to update it as frequent as possible)

A library for all types of components used in robots, including controlls for stepper motors, servo motors and more complex assemblies using said motors. Currently all implementations are made for the raspberry pi, though new implementations for more controllers are currently being made.

- [Getting started](docs/getting_started.md)

## Goal

- Create an all-in-one library to control motors, read sensors and do basic calculations in rust.
- Keep it as easy to use as possible
- Specialize the library for hobbyists and tinkerers
- Offer options for static aswell as dynamic typings

## In Action

Let us assume we want to control a simple stepper motor (in this example a [17HE15_1504_S](https://www.omc-stepperonline.com/index.php?route=product/product/get_file&file=2838/17HE15-1504S.pdf)) with a PWM controller connected to the BCM pins 27 and 19 (e.g. on a raspberry).

```rust ,ignore
use core::f32::consts::PI;

// Include the library
use syact::prelude::*;

// Pin declerations (BCM on raspberry pi)
const PIN_DIR : u8 = 27;
const PIN_STEP : u8 = 19;

// Define distance and max speed
const DELTA : Delta = Delta(2.0 * PI);
const OMEGA : Omega = Omega(10.0);

fn main() -> Result<(), syact::Error> {
    // Create the controls for a stepper motor
    let mut ctrl = Stepper::new(
      GenericPWM::new(PIN_STEP, PIN_DIR)?, // Controller
      StepperConst::MOT_17HE15_1504S    // Motor type
    );
    // Link the component to a system
    ctrl.write_data(CompData { 
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
      - [x] Preconfigured
      - [ ] With signals to be set by the controller
    - [ ] Inverting logical signals if desired
  - [x] [Servo motors](/docs/motors/servos.md)
  - [x] [DC motors](/docs/motors/dc_motors.md)
- [x] [Components](/docs/components.md)
  - [x] [Cylinder](/examples/cylinder.rs)
  - [x] Gear joint
  - [x] Cylinder-triangle
  - [x] [Conveyor](/examples/simple_conv/src/main.rs)
  - [x] DC-Motor
- [x] [Tools](/docs/tools.md)
  - [x] Tongs
  - [x] Axial joint
- [x] Calculation
  - [ ] Stepper motor curves
    - [ ] Low resolution
    - [x] High resolution
  - [ ] Paths
    - [x] Point-To-Point
    - [ ] Linear
  - [x] Overloads
  - [x] Forces
  - [x] Inertias
- [ ] Measurements
  - [x] Simple switch
  - [ ] Rotary resolver
  - [x] Trait for custom measurements
- [x] Extendable
  - [x] [Custom components](/docs/components.md#custom-components)
  - [x] Custom tools
  - [x] Custom meaurement systems
- [x] [Unit system](/docs/unit_system.md)
- [ ] Minimal
  - [ ] Fully supports `no_std` environment
  - [ ] Available for basic embedded systems
- [ ] [Platforms](/docs/platforms.md)
  - [x] Raspberry Pi and similar
  - [ ] Arduino

## Issues and requests

If you encounter any issues or if you have any request for new features, feel free to create an issue at the [GitHub repo](https://github.com/SamuelNoesslboeck/syact).
