use core::f32::consts::PI;

use plotters::prelude::*;
use crate::prelude::*;

#[test]
#[ignore = "Run manually"]
fn simple_curve() -> Result<(), Box<dyn std::error::Error>> {
    let consts = StepperConst::GEN;
    let mut vars = CompVars::ZERO;
    let data = StepperConfig::GEN;

    let delta = Delta(0.5 * (2.0 * PI));
    let omega_max = Omega(4.0 * PI);

    vars.inertia_load = Inertia(0.1);
    vars.f_bend = 0.1;

    let curve_time = curve::create_simple_curve(&consts, &vars, &data, delta, omega_max);
    let curve : Vec<Omega> = curve_time.iter().map(
        |time| { consts.omega(*time) }
    ).collect();
    
    let root = 
        SVGBackend::new("test/example-data/simple_curve.svg", (640, 480)).into_drawing_area();
    root.fill(&WHITE)?;
    let mut chart = ChartBuilder::on(&root)
        .caption(file!(), ("sans-serif", 50).into_font())
        .margin(5)
        .x_label_area_size(30)
        .y_label_area_size(30)
        .build_cartesian_2d( -1.0f32..(curve.len() as f32), -0.0f32..curve.iter().reduce(Omega::max_ref).unwrap().0)?;

    chart.configure_mesh().draw()?;

    chart
        .draw_series(LineSeries::new(
            (0..curve.len()).map(|index| (index as f32, curve[index].0)),
            &RED
        ))?;

    chart
        .draw_series(LineSeries::new(
            (0..curve_time.len()).map(|index| (index as f32, curve_time[index].0)),
            &GREEN
        ))?;

    root.present()?;
    
    Ok(())
}