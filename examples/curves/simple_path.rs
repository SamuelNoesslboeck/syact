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
    let dstack = vec![ Delta(1.0), Delta(2.0), Delta(1.0), Delta(3.0), Delta(4.0), Delta(0.5) ];
    let mut tstack = [ Time(1.0), Time(1.0), Time(1.0), Time(1.0), Time(1.0), Time(1.0) ];

    let mut nstack = vec![ PathNode::default(); dstack.len() ]; 

    for i in 0 .. dstack.len() {
        next(&mut tstack, &dstack, &mut nstack, &mut builder, i);
    }
}

fn next(tstack : &mut [Time], dstack : &[Delta], nstack : &mut [PathNode], builder : &mut CurveBuilder, i : usize) {
    let time = tstack[i];
    let delta = dstack[i];
    let omega_tar = delta / time;
    let ( _, fac ) = builder.next(delta, omega_tar);

    if (fac < 1.0) & builder.was_deccel() {
        let omega_err = builder.omega - omega_tar;
        let omega_cor = builder.omega_0 - omega_err;

        println!(" => {}: Fac: {}; Correct: {}, Error: {}, Builder: {}", i, fac, omega_cor, omega_err, builder.omega);

        tstack[i - 1] = dstack[i - 1] / omega_cor;
        builder.load_node(&nstack[i - 1]);
        next(tstack, dstack, nstack, builder, i - 1);

        // let (time_res, fac_res) =  builder.next(delta, omega_tar); 
        builder.next(delta, omega_tar);
    }

    nstack[i] = builder.get_node();
    println!("Omega_0: {}, Omega: {}, tar: {}", nstack[i].omega_0, builder.omega, omega_tar);
}

// fn correct(nodes : &[Delta], p_nodes : &mut Vec<PathNode>, builder : &mut CurveBuilder, t_mins : &mut [Time], index : usize) {
//     let omega_err = builder.omega - nodes[index] / t_mins[index]; // 2.0 * nodes[index] / t_mins[index] - builder.omega;
//     let omega_tar = p_nodes[index].omega_0 - omega_err; // 2.0 * nodes[index] / t_mins[index] - builder.omega; 
//     // println!(" => Omega: {}, Error: {}", omega_tar, omega_err);

//     // let omega_tar = builder.omega - 
//     // p_nodes.pop();
//     builder.load_node(&p_nodes[index - 1]);
//     let (_, c_fac) = builder.next(nodes[index - 1], omega_tar);
//     // p_nodes.push(builder.get_node());

//     if c_fac < 1.0 {
//         println!(" ... Correcting {}", c_fac);
//         correct(nodes, p_nodes, builder, t_mins, index - 1);
//     }

//     p_nodes.pop(); 

//     p_nodes.push(builder.get_node());

//     let node = nodes[index];

//     let (time, fac) = builder.next(node, node / t_mins[index]);
//     p_nodes.push(builder.get_node());
// }