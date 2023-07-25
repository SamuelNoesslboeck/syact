use syact::prelude::*;

fn omega_av<const N : usize>(deltas : &[Delta; N], times : &[Time; N]) -> [Omega; N] {
    let mut omegas = [Omega::ZERO; N];

    for i in 0 .. N {
        omegas[i] = deltas[i] / times[i];
    }

    omegas
}

fn omega_nodes<const N : usize>(omega_av : &[Omega; N], mut omega_0 : Omega) -> [Omega; N] {
    let mut omegas = [Omega::ZERO; N];

    for i in 0 .. N {
        omegas[i] = 2.0 * omega_av[i] - omega_0;
        omega_0 = omegas[i];
    }

    omegas
}

fn modify_omega_out<const N : usize>(omega_nodes : &mut [Omega; N], omega_av : &mut [Omega; N], deltas : &[Delta; N], times : &mut [Time; N], i : usize, o : Omega) {
    // Modify 
    omega_nodes[i] = o;

    if (i + 1) < N {
        // Not the last node   
        let next_node = omega_nodes[i + 1];
        let omega_av_new = (o + next_node) / 2.0;

        if omega_av_new.abs() > omega_av[i + 1].abs() {
            // Correct node velocity
            let av = omega_av[i + 1];
            let new_node = 2.0*av - o;

            modify_omega_out(omega_nodes, omega_av, deltas, times, i + 1, new_node);
        } else {
            // Correct average
            omega_av[i + 1] = omega_av_new;
            times[i + 1] = deltas[i + 1] / omega_av_new;
        }
    }

    if i > 0 {
        // Not the first node
        let last_node = omega_nodes[i - 1];
        let omega_av_new = (o + last_node) / 2.0;

        if omega_av_new.abs() > omega_av[i].abs() {
            // Correct node velocity
            let av = omega_av[i];
            let new_node = 2.0*av - o;

            modify_omega_out(omega_nodes, omega_av, deltas, times, i - 1, new_node);
        } else {
            // Correct average
            omega_av[i] = omega_av_new;
            times[i] = deltas[i] / omega_av_new;
        }
    }
}

fn alphas<const N : usize>(omegas : &[Omega; N], times : &[Time; N], mut omega_0 : Omega) -> [Alpha; N] {
    let mut alphas = [Alpha::ZERO; N]; 

    for i in 0 .. N {
        alphas[i] = (omegas[i] - omega_0) / times[i];
        omega_0 = omegas[i];
    }

    alphas
}

// fn recalc<const N : usize>(omega_av : &mut [Omega; N], omega_node : &mut [Omega; N], i : usize, omega : Omega) {
//     omega_node[i] = omega;
    
// }

fn main() -> Result<(), syact::Error> {
    let deltas = [ Delta(2.0), Delta(3.0), Delta(1.0) ];
    let mut times = [ Time(2.0), Time(1.5), Time(1.5) ];

    let mut omega_av = omega_av(&deltas, &times);
    let mut omega_nodes = omega_nodes(&omega_av, Omega::ZERO);

    modify_omega_out(&mut omega_nodes, &mut omega_av, &deltas, &mut times, 2, Omega::ZERO);

    let alphas = alphas(&omega_nodes, &times, Omega::ZERO);

    dbg!(deltas);
    dbg!(times);
    dbg!(omega_av);
    dbg!(omega_nodes);
    dbg!(alphas);

    Ok(())
}