use syact::prelude::*;

fn main() -> Result<(), syact::Error> {
    let deltas = [ Delta(2.0), Delta(3.0), Delta(-1.0), Delta(-2.0), Delta(2.0), Delta(2.0) ];
    let times = [ Time(2.0), Time(1.5), Time(1.5), Time(2.5), Time(2.0), Time(2.0) ];

    let mut builder = math::path::PathBuilder::new(times, deltas, Omega::ZERO);

    builder.gen_omega_av();
    builder.gen_omega_nodes(Omega::ZERO);
    builder.gen_alphas();

    builder.check_all_alphas(Alpha(0.5));

    dbg!(builder);

    Ok(())
}