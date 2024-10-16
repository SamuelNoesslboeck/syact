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
    /// Parameter that defines how much inaccuracy is allowed in time predictions etc.
    pub const PARAM_TIME_ACCURACY : f32 = 0.025;
// 