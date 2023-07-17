use serde::{Serialize, Deserialize};

/// Stores data for generic components 
#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct CompData {
    /// Supply voltage of the components in Volts
    pub u : f32,
    /// Safety factor of movements, sh
    pub s_f : f32
}

impl CompData {
    /// Generic `CompData` for testing purposes
    pub const GEN : Self = Self {
        u: 12.0,
        s_f: 1.5
    };

    /// Creates a new CompData instance
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

impl From<(f32, f32)> for CompData {
    fn from(data: (f32, f32)) -> Self {
        Self {
            u: data.0, 
            s_f: data.1
        }
    }
}