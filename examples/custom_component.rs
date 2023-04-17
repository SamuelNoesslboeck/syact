#![doc = "
The following example shows a custom component with a stepper motor as super component. 
Additionally it prints out a message every time write drive a relative distance.

### Note 

The cargo.toml file specified below is when running the example on a raspberry pi

```toml
# ...

[dependencies]
# Include the library configured for the raspberry pi
stepper_lib = { version = \"0.11.0\", features = [ \"rasp\" ] } 

# ...
```
"]

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

    fn drive_rel(&mut self, delta : Delta, omega : Omega) -> Result<Delta, stepper_lib::Error> {
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
        StepperCtrl::new(StepperConst::MOT_17HE15_1504S, PIN_DIR, PIN_STEP),
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