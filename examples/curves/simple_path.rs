use stepper_lib::StepperConst;
use stepper_lib::data::{CompVars, LinkedData};
use stepper_lib::math::CurveBuilder;
use stepper_lib::math::path::{PathBuilder, create_nodes, PathNode};
use stepper_lib::units::*;

fn main() {
    let consts = StepperConst::GEN;
    let mut vars = CompVars::ZERO;
    let lk = LinkedData::GEN;

    // let delta = Delta(0.5 * (2.0 * PI));
    // let omega_max = Omega(4.0 * PI);

    vars.j_load = Inertia(0.1);
    vars.f_bend = 0.1;

    let mut builder = CurveBuilder::new(&consts, &vars, &lk, Omega::ZERO);
    let nodes = vec![ Delta(1.0), Delta(2.0), Delta(1.0), Delta(3.0), Delta(4.0), Delta(0.5) ];
    let mut t_mins = [ Time(1.0), Time(1.0), Time(1.0), Time(1.0), Time(1.0), Time(1.0) ];

    let mut p_nodes = vec![
    ];

    for i in 0 .. nodes.len() {
        let node = nodes[i];

        let (time, fac) = builder.next(node, node / t_mins[i]);
        p_nodes.push(builder.get_node());
        println!("Time: {} | Fac: {} | Omega: {}", time, fac, builder.omega);

        if fac < 1.0 {
            if builder.was_deccel() {
                correct(&nodes, &mut p_nodes, &mut builder, &mut t_mins, i);
            }
        } 
    }
}

fn correct(nodes : &[Delta], p_nodes : &mut Vec<PathNode>, builder : &mut CurveBuilder, t_mins : &mut [Time], index : usize) {
    let omega_err = builder.omega - nodes[index] / t_mins[index]; // 2.0 * nodes[index] / t_mins[index] - builder.omega;
    let omega_tar = p_nodes[index].omega_0 - omega_err; // 2.0 * nodes[index] / t_mins[index] - builder.omega; 
    // println!(" => Omega: {}, Error: {}", omega_tar, omega_err);

    // let omega_tar = builder.omega - 
    // p_nodes.pop();
    builder.load_node(&p_nodes[index - 1]);
    let (_, c_fac) = builder.next(nodes[index - 1], omega_tar);
    // p_nodes.push(builder.get_node());

    if c_fac < 1.0 {
        println!(" ... Correcting {}", c_fac);
        correct(nodes, p_nodes, builder, t_mins, index - 1);
    }

    p_nodes.pop(); 

    p_nodes.push(builder.get_node());

    let node = nodes[index];

    let (time, fac) = builder.next(node, node / t_mins[index]);
    p_nodes.push(builder.get_node());
}