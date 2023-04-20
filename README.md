# stepper_lib

[![Crates.io version](https://img.shields.io/crates/v/stepper_lib.svg?style=flat-square) ](https://crates.io/crates/stepper_lib)
[![stepper_lib: rustc 1.68+]][Rust 1.68]

[stepper_lib: rustc 1.68+]: https://img.shields.io/badge/stepper_lib-rustc_1.68+-lightgray.svg
[Rust 1.68]: https://blog.rust-lang.org/2023/03/09/Rust-1.68.0.html

A library for all types of components used in robots, including controlls for stepper motors, servo motors and more complex assemblies using said motors. Currently all implementations are made for the raspberry pi, though new implementations for more controllers are currently being made.

Basis library for the [sybot_lib]("https://github.com/SamuelNoesslboeck/sybot_lib)

### Goal

- Create an all-in-one library to control motors, read sensors and do basic calculations in rust.

# In Action

Let us assume we want to control a simple stepper motor (in this example a [17HE15_1504_S](https://www.omc-stepperonline.com/index.php?route=product/product/get_file&file=2838/17HE15-1504S.pdf)) with a PWM controller connected to the BCM pins 27 and 19.

<details>
<summary>
Click to show Cargo.toml
</summary>

```toml
# ...

[dependencies]
# Include the library configured for the raspberry pi
stepper_lib = { version = "0.11.0", features = [ "rasp" ] } 

# ...
```
</details>
<p></p>

```rust
use core::f32::consts::PI;

// Include components and data
use stepper_lib::{StepperCtrl, StepperConst, SyncComp};
use stepper_lib::data::LinkedData;
// Include the unit system
use stepper_lib::units::*;

// Pin declerations (BCM on raspberry pi)
const PIN_DIR : u8 = 27;
const PIN_STEP : u8 = 19;

// Define distance and max speed
const DELTA : Delta = Delta(2.0 * PI);
const OMEGA : Omega = Omega(10.0);

fn main() -> Result<(), stepper_lib::Error> {
    // Create the controls for a stepper motor
    let mut ctrl = StepperCtrl::new(
        StepperConst::MOT_17HE15_1504S, 
        PIN_DIR, 
        PIN_STEP
    );
    // Link the component to a system
    ctrl.write_link(LinkedData { 
        u: 12.0,    // System voltage in volts
        s_f: 1.5    // System safety factor, should be at least 1.0
    }); 

    // Apply some loads
    ctrl.apply_inertia(Inertia(0.2));
    ctrl.apply_force(Force(0.10));

    println!("Staring to move");
    let delta_real = ctrl.drive_rel(DELTA, OMEGA)?;      // Move the motor
    println!("Distance {}rad with max speed {:?}rad/s done", 
        delta_real, OMEGA);

    Ok(())
}
```
(Source: "examples/stepper_motor.rs")

# Overview

- [Features](#features)
- [Components](#components)
- [Tools](#tools)
- [Platforms and simulation](#platforms-and-simulation)
- [Issues and requests](#issues-and-requests)

## Features

- [x] Motors
  - [x] Stepper motors
  - [x] [Servo motors](/docs/servos.md)
  - [x] [DC motors](/docs/dc_motors.md)
- [ ] Componentss
  - [x] Cylinder
  - [x] Gear joint
  - [x] Cylinder-triangle
  - [ ] Conveyor
- [x] Tools
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
- [x] Extendable
  - [x] Custom components
  - [x] Custom tools
- [ ] Minimal
  - [ ] Fully supports `no_std` environment
  - [ ] Available for basic embedded systems
- Platforms
  - Raspberry Pi and similar

## Components

A component (`trait SyncComp` in the library) represents a synchronous motor, optionally connected to a mechanical structure that transforms the rotatory movements created by the motor. If a struct implements the `SyncComp` trait, it can be linked into a group with `SyncCompGroup`, later required in the [sybot_lib](https://github.com/SamuelNoesslboeck/sybot_lib).

The library does include some standard components commonly used
- *Cylinder*, a simple cylinder translating the rotatory movements of a motor to a linear extension
- *GearJoint*, a motor connected to a gear that translates the movement with a certain ratio
- *Cylinder-triangle*, a cylinder being the hypotenuse in a triangular shape, creating a high torque/slow movement joint

In this example we drive a cylinder by a certain amount of millimeters.

<details>
<summary>Click to show Cargo.toml</summary>

```toml
# ...

[dependencies]
# Include the library configured for the raspberry pi
stepper_lib = { version = "0.11.0", features = [ "rasp" ] } 

# ...
```
</details>
<p></p>

```rust
// Include components and data
use stepper_lib::{StepperCtrl, StepperConst, SyncComp};
use stepper_lib::comp::Cylinder;
use stepper_lib::data::LinkedData;
// Include the unit system
use stepper_lib::units::*;

// Pin declerations (BCM on raspberry pi)
const PIN_DIR : u8 = 27;
const PIN_STEP : u8 = 19;

// Define distance and max speed
const DELTA : Delta = Delta(10.0);      // 10 millimeters
const OMEGA : Omega = Omega(20.0);      // 20 millimeters per second

fn main() -> Result<(), stepper_lib::Error> {
    // Create the controls for a stepper motor
    let mut cylinder = Cylinder::new(
        StepperCtrl::new(
            StepperConst::MOT_17HE15_1504S, 
            PIN_DIR, 
            PIN_STEP
        ),  1.273 // Spindle pitch of the cylinder, per radian the cylinder 
        // extends for 1.273 millimeters, this factor calculates out 
        // of the pitch per revolve (8mm) divided by 2*PI (for radians) 
    );
    // Link the component to a system
    cylinder.write_link(LinkedData { 
        u: 12.0,    // System voltage in volts
        s_f: 1.5    // System safety factor, should be at least 1.0
    }); 

    // Apply some loads
    cylinder.apply_inertia(Inertia(0.2));
    cylinder.apply_force(Force(0.10));

    println!("Staring to move ... ");
    let delta_real = cylinder.drive_rel(DELTA, OMEGA)?;         // Move the cylinder
    println!("Distance {}mm with max speed {:?}mm/s done", 
        delta_real, OMEGA);

    Ok(())
}
```

(Source: "examples/cylinder.rs")

### Custom components

If a component is desired that is not included in the standard components, then a custom component can be created. Simply implement the `SyncComp` trait for the component. 

There are two ways of defining a new component
- Defining a super component, which would be the motor to a gear or the cylinder in the `CylinderTriangle`, the only functions that have to be overwritten then are the functions required to communicate with the super component. Though in many cases some kind of ratio is added.
- Completely implementing the trait, therefore defining a completely new type of motor.

The following example shows a custom component with a stepper motor as super component. Additionally it prints out a message every time write drive a relative distance.

<details>
<summary>
Click to show Cargo.toml
</summary>

```toml
# ...

[dependencies]
# Include the library configured for the raspberry pi
stepper_lib = { version = "0.11.0", features = [ "rasp" ] } 

# ...
```
</details>
<p></p>

```rust
// Include components and data
use stepper_lib::{StepperCtrl, StepperConst, SyncComp};
use stepper_lib::data::LinkedData;
use stepper_lib::meas::SimpleMeas;
// Include the unit system
use stepper_lib::units::*;

// Pin declerations (BCM on raspberry pi)
const PIN_DIR : u8 = 27;
const PIN_STEP : u8 = 19;

// Define distance and max speed
const DELTA : Delta = Delta(10.0);      
const OMEGA : Omega = Omega(20.0);      

// Defining component structure
#[derive(Debug)]
struct MyComp {
    ctrl : StepperCtrl,
    ratio : f32
}

impl MyComp {
    pub fn new(ctrl : StepperCtrl, ratio : f32) -> Self {
        Self { ctrl, ratio }
    }
}

impl SimpleMeas for MyComp {
    fn init_meas(&mut self, _ : u8) {
        todo!()     // Not required in this example
    }
}

impl SyncComp for MyComp {
    // Required memebers
        fn vars<'a>(&'a self) -> &'a stepper_lib::data::CompVars {
            todo!()     // Not required in this example
        }

        fn link<'a>(&'a self) -> &'a LinkedData {
            todo!()     // Not required in this example
        }

        fn to_json(&self) -> Result<serde_json::Value, serde_json::Error> {
            todo!()     // Not required in this example
        }
    //
    
    // Super component (motor)
        // The following two overrides give the library access to the stepper motor controller stored in our component
        
        fn super_comp(&self) -> Option<&dyn SyncComp> {
            Some(&self.ctrl)
        }

        fn super_comp_mut(&mut self) -> Option<&mut dyn SyncComp> {
            Some(&mut self.ctrl)
        }
    // 

    // Ratio
        // The following two overrides cause the library to translate the distance by the ratio we defined for our component

        fn gamma_for_super(&self, this_gamma : Gamma) -> Gamma {
            this_gamma * self.ratio     // Distance is translated by the ratio
        }

        fn gamma_for_this(&self, super_gamma : Gamma) -> Gamma {
            super_gamma / self.ratio
        }
    // 

    fn drive_rel(&mut self, delta : Delta, omega : Omega) 
            -> Result<Delta, stepper_lib::Error> {
        println!("Now driving!"); // Our custom message

        let delta_real = self.ctrl.drive_rel(
            self.delta_for_super(delta, self.gamma()), 
            self.omega_for_super(omega, self.gamma())
        )?;

        Ok(self.delta_for_this(delta_real, self.gamma_for_super(self.gamma())))
    }
}

fn main() -> Result<(), stepper_lib::Error> {
    // Create the controls for a stepper motor
    let mut comp = MyComp::new(
        StepperCtrl::new(
            StepperConst::MOT_17HE15_1504S, 
            PIN_DIR, 
            PIN_STEP
        ),
        2.0 // Example ratio
    );
    // Link the component to a system
    comp.write_link(LinkedData { 
        u: 12.0,    // System voltage in volts
        s_f: 1.5    // System safety factor, should be at least 1.0
    }); 

    // Apply some loads
    comp.apply_inertia(Inertia(0.2));
    comp.apply_force(Force(0.10));

    println!("Staring to move ... ");
    let delta_real = comp.drive_rel(DELTA, OMEGA)?;         // Move the comp
    println!("Distance {}rad with max speed {:?}rad/s done", delta_real, OMEGA);

    Ok(())
}

// Console output: 
// "
// Starting to move ... 
// Now driving!
// Distance 10rad with max speed 20rad/s done
// "
```

(Source: "examples/custom_component.rs")

## Tools

The library also includes various simple tools like tongs, axial joints and so on. These tools are controlled by a servo controller, though other types of motors can be used in custom tools by implementing the `Tool` trait, just like `SyncComp`.

This example controls a simple pair of tongs using a servo motor.

<details>
<summary>
Click to show Cargo.toml
</summary>

```toml
# ...

[dependencies]
# Include the library configured for the raspberry pi
stepper_lib = { version = "0.11.0", features = [ "rasp" ] } 

# ...
```
</details>

```rust
use core::time::Duration;

use std::thread::sleep;

use stepper_lib::{SimpleTool, Tool};
use stepper_lib::comp::tool::Tongs;
use stepper_lib::ctrl::servo::ServoDriver;
use stepper_lib::data::servo::ServoConst;

// Include the unit system
use stepper_lib::units::*;

// Pin declerations (BCM on raspberry pi)
const PIN_PWM : u8 = 27;

fn main() -> Result<(), stepper_lib::Error> {
    // Create the tongs controller
    let mut tongs = Tongs::new(
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
```

## Platforms and simulation

The final goal of the library is to work on as many platforms as possible, even on embedded systems. To configure the library for a specific platform, the right features have to be enabled. Note that some of the features automatically enable `std` usage.

The current platforms and features enabled are
- "rasp": Raspberry Pi and similar controllers

```toml
# Platform features
rasp = [ "std", "dep:rppal" ]
```

If no platform is selected the library automatically goes into simulation mode. Meaning no movements will be actually executed, no pins will be written to or checked, which can lead to problems. As the library does for example not care if the same two pins are used in simulation mode.

## Issues and requests

If you encounter any issues or if you have any request for new features, feel free to create an issue at the [GitHub repo](https://github.com/SamuelNoesslboeck/stepper_lib).