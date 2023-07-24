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

fn recalc<const N : usize>(omega_av : &mut [Omega; N], omega_node : &mut [Omega; N], i : usize, omega : Omega) {
    omega_node[i] = omega;
    
}

fn main() -> Result<(), syact::Error> {
    let deltas = [ Delta(2.0), Delta(3.0), Delta(1.0) ];
    let times = [ Time(2.0), Time(1.5), Time(1.5) ];

    let omega_av = omega_av(&deltas, &times);
    let omega_nodes = omega_nodes(&omega_av, Omega::ZERO);

    dbg!(deltas);
    dbg!(times);
    dbg!(omega_av);
    dbg!(omega_nodes);

    Ok(())
}