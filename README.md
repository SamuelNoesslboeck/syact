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
- [Examples](docs/examples.md)

## Goal

- Create an all-in-one library to control motors, read sensors and do basic calculations in rust.
- Keep it as easy to use as possible
- Specialize the library for hobbyists and tinkerers
- Offer options for static aswell as dynamic typings

## In Action

Let us assume we want to control a simple stepper motor (in this example a [17HE15_1504_S](https://www.omc-stepperonline.com/index.php?route=product/product/get_file&file=2838/17HE15-1504S.pdf)) with a PWM controller connected to the BCM pins 27 and 19 (e.g. on a raspberry).

```rust ,ignore
TODO: NEW EXAMPLE
```

## Features

- [x] Motors
  - [x] Stepper motors
    - [x] Absolute/relative movements
    - [x] Continuous movements
    - [ ] Microstepping
      - [x] Preconfigured
      - [ ] With signals to be set by the controller
    - [ ] Inverting logical signals if desired
  - [x] Servo motors
  - [x] DC motors
- [x] Components
  - [x] Cylinder
  - [x] Gear joint
  - [x] Cylinder-triangle
  - [x] Conveyor
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
- [x] [Platforms](/docs/platforms.md)
  - [x] Raspberry Pi and similar
  
## Issues and requests

If you encounter any issues or if you have any request for new features, feel free to create an issue at the [GitHub repo](https://github.com/SamuelNoesslboeck/syact).
