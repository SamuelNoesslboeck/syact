use crate::prelude::*;
use crate::math::PathNode;

#[test]
#[ignore = "Run manually"]
fn complex_path() {
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
    let ( time_real, fac ) = builder.next(delta, omega_tar);

    if fac < 1.0 {
        if builder.was_deccel() {
            let omega_err = builder.omega - omega_tar;
            let omega_cor = builder.omega_0 - omega_err;
    
            println!(" => {}: Fac: {}; Correct: {}, Error: {}, Builder: {}", i, fac, omega_cor, omega_err, builder.omega);
    
            tstack[i - 1] = dstack[i - 1] / omega_cor;
            builder.load_node(&nstack[i - 1]);
            next(tstack, dstack, nstack, builder, i - 1);
    
            // let (time_res, fac_res) =  builder.next(delta, omega_tar); 
            builder.next(delta, omega_tar);
        } else {
            tstack[i] = time_real;
        }
    }

    nstack[i] = builder.get_node();
    println!("Omega_0: {}, Omega: {}, tar: {}", nstack[i].omega_0, builder.omega, omega_tar);
}
