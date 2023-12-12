// ####################
// #    SUBMODULES    #
// ####################
    mod comp;

    /// Crate for servo motor data
    pub mod servo;

    pub mod stepper;
    use core::ops::Mul;
    use core::str::FromStr;

    use serde::{Serialize, Deserialize};
    pub use stepper::{StepperConfig, StepperConst};

    /// Crate for variables read and written during runtime
    mod var;
    pub use var::ActuatorVars;
//

#[derive(Clone, Copy, Debug, PartialEq, PartialOrd, Serialize, Deserialize)]
pub struct SpeedFactor(f32);

impl SpeedFactor {
    pub const MAX : Self = Self(1.0);
    pub const HALF : Self = Self(0.5);
}

impl SpeedFactor {
    pub fn try_from(value : f32) -> Option<Self> {
        if (value <= 1.0) & (value > 0.0) {
            Some(Self(value))
        } else {
            None
        }
    }

    pub const unsafe fn from_unchecked(value : f32) -> Self {
        Self(value)
    }
}

impl From<f32> for SpeedFactor {
    fn from(value: f32) -> Self {
        if (value <= 1.0) & (value > 0.0) {
            Self(value)
        } else {
            panic!("Invalid float for speed factor! ({value})\nValue must not be greater than 1, nor 0 or less")
        }
    }
}

impl From<SpeedFactor> for f32 {
    fn from(value: SpeedFactor) -> Self {
        value.0
    }
}

impl FromStr for SpeedFactor {
    type Err = <f32 as FromStr>::Err;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        Ok(Self::from(f32::from_str(s)?))
    }
}

impl Default for SpeedFactor {
    fn default() -> Self {
        Self(1.0)
    }
}

impl Mul<SpeedFactor> for SpeedFactor {
    type Output = SpeedFactor;

    fn mul(self, rhs: SpeedFactor) -> Self::Output {
        SpeedFactor(self.0 * rhs.0)
    }
}

impl Mul<f32> for SpeedFactor {
    type Output = SpeedFactor;

    fn mul(self, rhs: f32) -> Self::Output {
        SpeedFactor::from(self.0 * rhs)
    }
}

impl Mul<SpeedFactor> for f32 {
    type Output = SpeedFactor;

    fn mul(self, rhs: SpeedFactor) -> Self::Output {
        SpeedFactor::from(self * rhs.0)
    }
}


#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct MicroSteps(u8);

impl MicroSteps {
    pub fn as_u8(self) -> u8 {
        self.0
    }
}

impl From<u8> for MicroSteps {
    fn from(value: u8) -> Self {
        if (value & value.wrapping_sub(1)) == 0 {   // Check if power of 2
            Self(value)
        } else {
            panic!("")
        }
    }
}

impl FromStr for MicroSteps {
    type Err = <u8 as FromStr>::Err;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        Ok(Self(u8::from_str(s)?))
    }
}

impl From<MicroSteps> for u8 {
    fn from(value: MicroSteps) -> Self {
        value.0
    }
}

impl Default for MicroSteps {
    fn default() -> Self {
        Self(1)
    }
}