use core::f32::consts::PI;

use plotters::prelude::*;
use crate::prelude::*;

#[test]
#[ignore = "Run manually"]
fn curve_to_speed() -> Result<(), Box<dyn std::error::Error>> {
    let consts = StepperConst::GEN;
    let mut vars = CompVars::ZERO;
    let data = CompData::GEN;

    // let delta = Delta(0.5 * (2.0 * PI));
    let omega_max = Omega(4.0 * PI);

    vars.j_load = Inertia(0.000_25);
    vars.f_bend = 0.1;

    let mut builder = CurveBuilder::new(&consts, &vars, &data, Omega::ZERO);
    let mut curve : Vec<Omega> = builder.to_speed(omega_max * 0.5)?.iter().map(
        |t| consts.omega(*t)
    ).collect();
    
    if curve.len() < 100 {
        curve.append(&mut vec![omega_max * 0.5; 100 - curve.len()]);
    }

    curve.append(&mut builder.to_speed(omega_max * 0.8)?.iter().map(
        |t| consts.omega(*t)
    ).collect());

    curve.append(&mut builder.to_speed(omega_max * 0.2)?.iter().map(
        |t| consts.omega(*t)
    ).collect());

    
    let root = SVGBackend::new(
        "test/example-data/curve_to_speed.svg", (640, 480)).into_drawing_area();
    root.fill(&WHITE)?;
    let mut chart = ChartBuilder::on(&root)
        .caption(file!(), ("sans-serif", 50).into_font())
        .margin(5)
        .x_label_area_size(30)
        .y_label_area_size(30)
        .build_cartesian_2d( -1.0f32..(curve.len() as f32), 
            -0.0f32..curve.iter().reduce(Omega::max_ref).unwrap().0)?;

    chart.configure_mesh().draw()?;

    chart
        .draw_series(LineSeries::new(
            (0..curve.len()).map(|index| (index as f32, curve[index].0)),
            &RED
        ))?;

    root.present()?;
    
    Ok(())
}