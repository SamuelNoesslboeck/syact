use crate::prelude::*;

#[test]
#[ignore = "Run manually"]
fn simple_path() {
    let consts = StepperConst::GEN;
    let mut vars = CompVars::ZERO;
    let lk = LinkedData::GEN;

    // let delta = Delta(0.5 * (2.0 * PI));
    // let omega_max = Omega(4.0 * PI);

    vars.j_load = Inertia(0.1);
    vars.f_bend = 0.1;
    
    let mut builder = PathBuilder::new([
            CurveBuilder::new(&consts, &vars, &lk, Omega::ZERO),
            CurveBuilder::new(&consts, &vars, &lk, Omega::ZERO)
    ]);

    let mut dstack = vec![ ];
    let mut tstack = vec![ ];

    for i in 0 .. 5 {
        for n in 0 .. (6 - i) {
            dstack.push([ -Delta(1.0 + i as f32 * 0.5 + n as f32 * 0.6), -Delta(5.0) ] );
            tstack.push(Time(1.0));
        }
    }

    let tstack_copy = tstack.clone();

    builder.generate(&mut tstack, &dstack);

    println!("Omega_tar: ["); 

    for i in 0 .. dstack.len() {
        let mut i2 = i + 1;

        if i2 == dstack.len() {
            i2 = i2 - 1;
        }

        println!("   [ {}, {}, {}, {}, {}, {} ],", 
            dstack[i][0] / tstack_copy[i], 
            builder.get_node(0, i2).omega_0, 
            dstack[i][1] / tstack_copy[i], 
            builder.get_node(1, i2).omega_0,
            dstack[i][0] / tstack_copy[i] / (dstack[i][1] / tstack_copy[i]),
            builder.get_node(0, i2).omega_0 / builder.get_node(1, i2).omega_0
        );
    }

    println!("]");
}
