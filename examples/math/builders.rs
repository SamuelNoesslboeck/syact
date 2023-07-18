/// ########################
/// ##      Builders      ##
/// ########################
/// 
/// This example features builders, the `Iterator`-based curve generating structures that ensure save movements of stepper motors and more.  
/// There are different levels of Iterators with different properties that will be showcased in the following example

// Simple include of many library features
use syact::{prelude::*, math::CtrlStepTimeBuilder};

fn main() -> Result<(), syact::Error> {
    // Initialize a new stepper motor with generic values (simple NEMA 17 motor with 12V ...)
    let mut stepper = Stepper::new_sim();
    stepper.write_data(CompData::GEN);
    stepper.setup()?;

    println!("########################");
    println!("##      Builders      ##");
    println!("########################");
    println!("");
    println!("# Units");
    println!("- Time in seconds");
    println!("- Speed in steps/second");

    // #######################
    // #   StepTimeBuilder   #
    // #######################
    // 
    // This builder accelerates until it reaches the maximum speed given at it's creation (it can of course be modified)
        println!("\n[StepTimeBuilder - Unloaded]");

        // Create a new `StepTimeBuilder`
        let builder = stepper.create_builder(
            Omega::ZERO,    // The start velocity of the component
            stepper.omega_max()     // The maximum velocity of the component
        ); 

        for (index, step) in builder.enumerate() {
            println!("- [{}] Time: {}, Speed: {}", index, step, 1.0 / step);
        }


        println!("\n[StepTimeBuilder - Loaded]");

        stepper.apply_inertia(Inertia(0.0001));

        // Create a new `StepTimeBuilder`
        let builder = stepper.create_builder(
            Omega::ZERO,    // The start velocity of the component
            stepper.omega_max()     // The maximum velocity of the component
        ); 

        for (index, step) in builder.enumerate() {
            println!("- [{}] Time: {}, Speed: {}", index, step, 1.0 / step);
        }
    // 

    // ###########################
    // #   CtrlStepTimeBuilder   #
    // ###########################
    //
    // This builder accelerates to a given speed omega_tar,
    // the omega_max makes sure the component is not targeting a speed higher than it's capacity
        println!("\n[CtrlStepTimeBuilder - Loaded]");

        stepper.apply_inertia(Inertia(0.0001));

        println!("To 30rad/s ... ");

        let mut builder = CtrlStepTimeBuilder::from_builder(
            stepper.create_builder(Omega::ZERO, stepper.omega_max())
        );

        builder.set_omega_tar(Omega(30.0))?;

        for (index, step) in builder.enumerate() {
            println!("- [{}] Time: {}, Speed: {}", index, step, 1.0 / step);
        }

        println!("To 10rad/s ... ");

        let mut builder = CtrlStepTimeBuilder::from_builder(
            stepper.create_builder(Omega(30.0), stepper.omega_max())
        );

        builder.set_omega_tar(Omega(10.0))?;

        for (index, step) in builder.enumerate() {
            println!("- [{}] Time: {}, Speed: {}", index, step, 1.0 / step);
        }
    // 

    Ok(())
}