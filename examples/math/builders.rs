//! ########################
//! ##      Builders      ##
//! ########################
//! 
//! This example features builders, the `Iterator`-based curve generating structures that ensure save movements of stepper motors and more.  
//! There are different levels of Iterators with different properties that will be showcased in the following example

use std::time::Instant;

// Simple include of many library features
use syact::{prelude::*, math::{HRCtrlStepBuilder, HRLimitedStepBuilder, HRStepBuilder}};

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
        // [Unloaded]
            println!("\n[StepTimeBuilder - Unloaded]");

            // Create a new `StepTimeBuilder`
            let builder = HRStepBuilder::from_motor(
                &stepper,       // The stepper motor
                Omega::ZERO    // The start velocity of the component
            ); 

            // Iterate through
            for (index, step) in builder.enumerate() {
                println!("- [{}] Time: {}, Speed: {}", index, step, 1.0 / step);
            }
        // 

        // [Loaded]
            println!("\n[StepTimeBuilder - Loaded]");

            // Apply the load to the stepper motor
            stepper.apply_inertia(Inertia(0.0001));

            // Create a new `StepTimeBuilder`
            let builder = HRStepBuilder::from_motor(
                &stepper,       // Motor 
                Omega::ZERO     // Start velocity
            );

            // Iterate through
            for (index, step) in builder.enumerate() {
                println!("- [{}] Time: {}, Speed: {}", index, step, 1.0 / step);
            }
        // 
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

        let mut builder = HRCtrlStepBuilder::from_builder(
            HRStepBuilder::from_motor(
                &stepper,           // The stepper motor
                Omega::ZERO       // The start velocity of the component
            )
        );

        builder.set_omega_tar(Omega(30.0))?;

        for (index, step) in builder.enumerate() {
            println!("- [{}] Time: {}, Speed: {}", index, step, 1.0 / step);
        }

        println!("To 10rad/s ... ");

        let mut builder = HRCtrlStepBuilder::from_builder(
            HRStepBuilder::from_motor(
                &stepper,       // The stepper motor
                Omega::ZERO    // The start velocity of the component
            )
        );

        builder.set_omega_tar(Omega(10.0))?;

        for (index, step) in builder.enumerate() {
            println!("- [{}] Time: {}, Speed: {}", index, step, 1.0 / step);
        }
    // 

    // ###############################
    // #   LimitedStepTimeBuilder    #
    // ###############################
        println!("\n[LimitedStepTimeBuilder - Predicting]");

        stepper.apply_inertia(Inertia(0.001));

        let delta = Delta(120.0);
        let speed_f = 1.0;

        println!("Bad prediction: {}", delta / stepper.omega_max() / speed_f);

        let bench = Instant::now();
        let time = stepper.approx_time_ptp(delta, speed_f, 0)?;
        let el = bench.elapsed().as_secs_f32();

        println!("Predicted time: {}; Calculation time: {}", time, el);

        let bench = Instant::now();
        let mut builder = HRLimitedStepBuilder::from_builder(
            HRStepBuilder::from_motor(
                &stepper,       // The stepper motor
                Omega::ZERO    // The start velocity of the component
            )
        );

        builder.set_omega_tar(stepper.omega_max() * speed_f)?;
        builder.set_steps_max(stepper.consts().steps_from_ang_abs(delta, stepper.micro()));

        let sum : f32 = builder.map(|t| t.0).sum();

        let el = bench.elapsed().as_secs_f32();
        println!("Iterated time: {}, Calculation time: {}", sum, el);


        println!("\n[LimitedStepTimeBuilder - Scaling]");

        let scale = 0.5;

        let mut builder = HRLimitedStepBuilder::from_builder(
            HRStepBuilder::from_motor(
                &stepper,       // The stepper motor
                Omega::ZERO    // The start velocity of the component
            )
        );

        builder.set_omega_tar(stepper.omega_max())?;
        builder.set_steps_max(stepper.consts().steps_from_ang_abs(delta, stepper.micro()));

        let sum : f32 = builder.map(|t| t.0).sum();

        println!("Iterated time: {}", sum);

        let mut builder = HRLimitedStepBuilder::from_builder(
            HRStepBuilder::from_motor(
                &stepper,       // The stepper motor
                Omega::ZERO    // The start velocity of the component
            )
        );

        builder.set_omega_tar(stepper.omega_max())?;
        builder.set_speed_f(scale);
        builder.set_steps_max(stepper.consts().steps_from_ang_abs(delta, stepper.micro()));

        let sum_scaled : f32 = builder.map(|t| t.0).sum();

        println!("Iterated time: {}, Scale: {}, Calc-Scale: {}", sum_scaled, scale, sum / sum_scaled);
    //

    Ok(())
}