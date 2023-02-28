use serde::{Serialize, Deserialize};

#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct LinkedData 
{
    pub u : f32,
    pub s_f : f32
}

impl LinkedData 
{
    pub const EMPTY : Self = Self {
        u : 0.0, 
        s_f : 0.0
    };

    pub const GEN : Self = Self {
        u: 12.0,
        s_f: 1.5
    };
}

impl From<(f32, f32)> for LinkedData 
{
    fn from(data: (f32, f32)) -> Self {
        Self {
            u: data.0, 
            s_f: data.1
        }
    }
}