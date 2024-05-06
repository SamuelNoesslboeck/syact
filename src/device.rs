// # `device`-module
//
// Stable: 0.12.1

// ####################
// #    SUBMODULES    #
// ####################
    /// Basic DC-Motors
    // pub mod dc_motor;
    // pub use dc_motor::DcMotor;

    // /// Helper functions and structs for deserializing
    // mod des;

    /// LEDs and other light sources
    mod led;
    pub use led::{LED, LED_PWM_FREQ};

    /// PWM-signal 
    mod pwm;
    pub use pwm::SoftwarePWM;

    /// Structs and methods for basic servo motors
    mod servo;
    pub use servo::Servo;
//