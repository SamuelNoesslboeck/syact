// Rules
// #[cfg(not(feature = "testing"))]
// compile_error!("Compile tests with 'testing' feature!");

// ####################
// #    SUBMODULES    #
// ####################
    mod sync;
    #[allow(unused)]
    pub use sync::{Stepper, ComplexStepper, SimulatedController};

    mod data;
// 

// ####################
// #    PARAMETERS    #
// ####################
    /// Parameter that defines how much inaccuracy is allowed in time predictions etc.
    pub const PARAM_TIME_ACCURACY : f32 = 0.05;
// 