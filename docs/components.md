# Components

A component (`trait SyncComp` in the library) represents a synchronous motor, optionally connected to a mechanical structure that transforms the rotatory movements created by the motor. If a struct implements the `SyncComp` trait, it can be linked into a group with `SyncCompGroup`, later required in the [sybot_lib](https://github.com/SamuelNoesslboeck/sybot_lib).

The library does include some standard components commonly used

- *Cylinder*, a simple cylinder translating the rotatory movements of a motor to a linear extension
- *Gear*, a motor connected to a gear that translates the movement with a certain ratio
- *Cylinder-triangle*, a cylinder being the hypotenuse in a triangular shape, creating a high torque/slow movement joint

## Example 

In this example we drive a cylinder by a certain amount of millimeters.

<details>
<summary>Click to show Cargo.toml</summary>

### Note

The cargo.toml file specified below is when running the example on a raspberry pi

```toml
# ...

[dependencies]
# Include the library configured for the raspberry pi
syact = { version = \"0.12.1\", features = [ \"rasp\" ] } 

# ...
```

</details>
<p></p>

```rust ,ignore
// Include the library
use syact::prelude::*;

// Pin declerations (BCM on raspberry pi)
const PIN_DIR : u8 = 27;
const PIN_STEP : u8 = 19;

// Define distance and max speed
const DELTA : Delta = Delta(10.0);      // 10 millimeters
const OMEGA : Omega = Omega(20.0);      // 20 millimeters per second

fn main() -> Result<(), syact::Error> {
    // Create the controls for a stepper motor
    let mut cylinder = Cylinder::new(
        Stepper::new(GenericPWM::new(PIN_STEP, PIN_DIR)?, StepperConst::MOT_17HE15_1504S),
        1.273       // Spindle pitch of the cylinder, per radian the cylinder extends for 1.273 millimeters,
                    // this factor calculates out of the pitch per revolve (8mm) divided by 2*PI (for radians) 
    );
    // Link the component to a system
    cylinder.write_data(CompData { 
        u: 12.0,    // System voltage in volts
        s_f: 1.5    // System safety factor, should be at least 1.0
    }); 
    cylinder.setup()?;

    // Apply some loads
    cylinder.apply_inertia(Inertia(0.2));
    cylinder.apply_force(Force(0.10));

    println!("Staring to move ... ");
    let delta_real = cylinder.drive_rel(DELTA, 1.0)?;         // Move the cylinder
    println!("Distance {}mm with max speed {:?}mm/s done", delta_real, OMEGA);

    Ok(())
}
```

(See [the rust example](../examples/cylinder.rs))

## Custom components

If a component is desired that is not included in the standard components, then a custom component can be created. Simply implement the `SyncComp` trait for the component.

There are two ways of defining a new component

- Defining a parent component, which would be the motor to a gear or the cylinder in the `CylinderTriangle`, the only functions that have to be overwritten then are the functions required to communicate with the parent component. Though in many cases some kind of ratio is added.
- Completely implementing the trait, therefore defining a completely new type of motor.

The following example shows a custom component with a stepper motor as parent component. Additionally it prints out a message every time write drive a relative distance.

<details>
<summary>
Click to show Cargo.toml
</summary>

```toml
# ...

[dependencies]
# Include the library configured for the raspberry pi
syact = { version = "0.12.1", features = [ "rasp" ] } 

# ...
```

</details>
<p></p>

```rust ,ignore

// Include library
use syact::prelude::*;

// Pin declerations (BCM on raspberry pi)
const PIN_DIR : u8 = 27;
const PIN_STEP : u8 = 19;

// Define distance and max speed
const DELTA : Delta = Delta(10.0);      
const OMEGA : Omega = Omega(10.0);      

// Defining component structure
#[derive(Debug)]
struct MyComp {
    ctrl : Stepper,     // The stepper motor built into the component
    ratio : f32             // The gear ratio, e.g. a spingle or
}

impl MyComp {
    pub fn new(ctrl : Stepper, ratio : f32) -> Self {
        Self { ctrl, ratio }
    }
}

impl Setup for MyComp {
    fn setup(&mut self) -> Result<(), syact::Error> {
        self.ctrl.setup()?;  // Setting up the parent component
        Ok(())      
    }
}

impl SyncComp for MyComp {
    // Required memebers
        fn vars<'a>(&'a self) -> &'a syact::data::CompVars {
            todo!()     // Not required in this example
        }

        fn data<'a>(&'a self) -> &'a CompData {
            todo!()     // Not required in this example
        }
    //
    
    // Super component (motor)
        // The following two overrides give the library access to the stepper motor controller stored in our component
        fn parent_comp(&self) -> Option<&dyn SyncComp> {
            Some(&self.ctrl)
        }

        fn parent_comp_mut(&mut self) -> Option<&mut dyn SyncComp> {
            Some(&mut self.ctrl)
        }
    // 

    // Ratio
        // The following two overrides cause the library to translate the distance by the ratio we defined for our component
        fn gamma_for_parent(&self, this_gamma : Gamma) -> Gamma {
            this_gamma * self.ratio     // Distance is translated by the ratio
        }

        fn gamma_for_this(&self, parent_gamma : Gamma) -> Gamma {
            parent_gamma / self.ratio
        }
    // 

    fn drive_rel(&mut self, delta : Delta, speed_f : f32) -> Result<Delta, syact::Error> {
        println!("Now driving!"); // Our custom message

        let delta_real = self.ctrl.drive_rel(
            self.delta_for_parent(delta, self.gamma()), 
            speed_f
        )?;

        Ok(self.delta_for_this(delta_real, self.gamma_for_parent(self.gamma())))
    }
}

fn main() -> Result<(), syact::Error> {
    // Create the controls for a stepper motor
    let mut comp = MyComp::new(
        Stepper::new(GenericPWM::new(PIN_STEP, PIN_DIR)?, StepperConst::MOT_17HE15_1504S),
        2.0 // Example ratio
    );
    // Link the component to a system
    comp.write_data(CompData { 
        u: 12.0,    // System voltage in volts
        s_f: 1.5    // System safety factor, should be at least 1.0
    }); 

    comp.setup()?;

    // Apply some loads
    comp.apply_inertia(Inertia(0.2));
    comp.apply_force(Force(0.10));
    
    // Limit the component velocity
    comp.set_omega_max(OMEGA);

    println!("Staring to move ... ");
    let delta_real = comp.drive_rel(DELTA, 1.0)?;         // Move the comp
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

(See [the rust example](../examples/custom_component.rs))
