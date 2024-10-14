// ####################
// #    SUBMODULES    #
// ####################
    mod act;
    #[allow(unused)]
    pub use act::{Stepper, ComplexStepper, SimulatedController};

    mod data;
    mod math;
// 

// ####################
// #    PARAMETERS    #
// ####################
    pub const PARAM_TIME_ACCURACY : f32 = 0.025;
// 