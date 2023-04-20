use crate::comp::asyn::{Direction, AsyncComp};
use crate::ctrl::pwm::PWMOutput;
use crate::units::Omega;

/// A simple dc motor with two pins as PWM control
pub struct DcMotor {
    sig_cw : PWMOutput,
    sig_ccw : PWMOutput,

    dir : Direction,
    speed_f : f32,

    freq : Omega
}

impl DcMotor {
    /// Creates a new instance of a `DcMotor` with the following parameters
    /// - `pin_cw` is the pin for driving the motor in the clockwise direction
    /// - `pin_ccw` is the pin for driving the motor in the counter-clockwise direction
    /// - `freq` defines the PWM-frequency for 
    pub fn new(pin_cw : u8, pin_ccw : u8, freq : Omega) -> Self {
        Self {
            sig_cw: PWMOutput::new(pin_cw),
            sig_ccw: PWMOutput::new(pin_ccw),

            dir: Direction::None,
            speed_f: 0.0,

            freq
        }
    }

    /// Returns the frequency of the PWM signal for the motor
    #[inline(always)]
    pub fn freq(&self) -> Omega {
        self.freq
    }
}

impl AsyncComp for DcMotor {
    fn setup(&mut self) {
        self.sig_cw.start();
        self.sig_ccw.start();
    }

    fn drive(&mut self, dir : Direction, speed_f : f32) -> Result<(), crate::Error> {
        #[cfg(feature = "std")]
        if (1.0 < speed_f) & (speed_f < 0.0) {
            panic!("Bad speed factor! {}", speed_f);
        }
    
        self.dir = dir;
        self.speed_f = speed_f;

        match self.dir {
            Direction::CW => {
                self.sig_cw.set_freq(self.freq, speed_f);
                self.sig_ccw.set_freq(self.freq, 0.0);
            },
            Direction::CCW => {
                self.sig_cw.set_freq(self.freq, 0.0);
                self.sig_ccw.set_freq(self.freq, speed_f);
            },
            Direction::None => {
                self.sig_cw.set_freq(self.freq, 0.0);
                self.sig_ccw.set_freq(self.freq, 0.0);
            }
        };

        Ok(())
    }

    #[inline(always)]
    fn dir(&self) -> Direction {
        self.dir
    }

    #[inline(always)]
    fn speed_f(&self) -> f32 {
        self.speed_f
    }
}