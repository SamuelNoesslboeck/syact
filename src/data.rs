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

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
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
            panic!("Number of microsteps must be a power of 2! ({} given)", value)
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

impl Mul<MicroSteps> for u64 {
    type Output = u64;

    fn mul(self, rhs: MicroSteps) -> Self::Output {
        self * (rhs.as_u8() as u64)
    }
}