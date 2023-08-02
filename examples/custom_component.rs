#![doc = "
The following example shows a custom component with a stepper motor as parent component. 
Additionally it prints out a message every time write drive a relative distance.

### Note 

The cargo.toml file specified below is when running the example on a raspberry pi

```toml
# ...

[dependencies]
syact = { version = \"0.12.1\", features = [ \"rasp\" ] } 

# ...
```
"]

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