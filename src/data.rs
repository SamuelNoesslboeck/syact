// ####################
// #    SUBMODULES    #
// ####################
    mod comp;

    /// Crate for servo motor data
    pub mod servo;

    pub mod stepper;
    pub use stepper::{StepperConfig, StepperConst};

    /// Crate for variables read and written during runtime
    mod var;
    pub use var::ActuatorVars;
//