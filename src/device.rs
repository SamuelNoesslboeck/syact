// # `device`-module
//
// Stable: 0.12.1

// ####################
// #    SUBMODULES    #
// ####################
    /// LEDs and other light sources
    mod led;
    pub use led::{LED, LED_PWM_FREQ};

    /// Structs and methods for basic servo motors
    mod servo;
    pub use servo::MiniServo;
//