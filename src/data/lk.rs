use serde::{Serialize, Deserialize};

/// Stores data that is the same for multiple components in many cases, such as *supply voltage* 
#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct LinkedData {
    /// Supply voltage of the components in Volts
    pub u : f32,
    /// Safety factor of movements, sh
    pub s_f : f32
}

impl LinkedData {
    /// Generic `LinkedData` for testing purposes
    pub const GEN : Self = Self {
        u: 12.0,
        s_f: 1.5
    };

    /// Creates a new LinkedData instance
    /// 
    /// # Panics
    /// 
    /// Panics if the given safety factor `s_f` 
    #[inline(always)]
    pub fn new(u : f32, s_f : f32) -> Self {
        if s_f < 1.0 {
            panic!("The given safetly factor is invalid! {}", s_f);
        }

        Self { u, s_f }
    }
}

impl From<(f32, f32)> for LinkedData {
    fn from(data: (f32, f32)) -> Self {
        Self {
            u: data.0, 
            s_f: data.1
        }
    }
}