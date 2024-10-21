// ####################
// #    SUBMODULES    #
// ####################
    /// Servo motor data
    pub mod servo;
    
    /// All data and parameters related to stepper motors
    pub mod stepper;
    pub use stepper::{StepperConfig, StepperConst, MicroSteps};

    /// Crate for variables read and written during runtime
    mod var;
    pub use var::ActuatorVars;
//