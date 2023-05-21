use plotters::prelude::*;
use crate::prelude::*;

#[test]
#[ignore = "Run manually"]
fn torque_curve() -> Result<(), Box<dyn std::error::Error>> {
    let consts = StepperConst::GEN;
    let mut vars = CompVars::ZERO;
    let lk = LinkedData::GEN;

    let scalar : f32 = 10.0;

    vars.j_load = Inertia(5.0);
    vars.f_bend = 0.1;

    let curve : Vec<Force> = (0 .. 1000).map(
        |index| force::torque_dyn(&consts, Omega(index as f32 / scalar), lk.u)).collect();
    
    let root = 
        SVGBackend::new("test/example-data/torque_curve.svg", (640, 480)).into_drawing_area();
    root.fill(&WHITE)?;
    let mut chart = ChartBuilder::on(&root)
        .caption("Stepper-Curve", ("sans-serif", 50).into_font())
        .margin(5)
        .x_label_area_size(30)
        .y_label_area_size(30)
        .build_cartesian_2d( 0.0f32..(curve.len() as f32 / scalar), 0.0f32..curve.iter().reduce(Force::max_ref).unwrap().0)?;

    chart.configure_mesh().draw()?;

    chart
        .draw_series(LineSeries::new(
            (0..curve.len()).map(|index| (index as f32 / scalar, curve[index].0)),
            &RED
        ))?
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &RED));

    chart
        .configure_series_labels()
        .background_style(&WHITE.mix(0.8))
        .border_style(&BLACK)
        .draw()?;

    root.present()?;

    // Console output
    println!(" -> Tau: {}", consts.tau(lk.u));
    
    Ok(())
}