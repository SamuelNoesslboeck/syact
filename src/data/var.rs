use crate::units::*;

#[derive(Debug, Clone, Default)]
pub struct Limits {
    /// Limit for minimum gamma distance
    pub min : Option<Gamma>,  
    /// Limit for maximum gamma distance
    pub max : Option<Gamma>
}

impl Limits {
    const NONE : Self = Self { min: None, max: None };
}

#[derive(Clone, Debug, Default)]
pub struct CompVars {
    pub t_load : Force,
    pub j_load : Inertia,

    pub f_bend : f32,

    pub lim : Limits
}

impl CompVars  {
    pub const ZERO : Self = Self { 
        t_load: Force::ZERO, 
        j_load: Inertia::ZERO, 

        f_bend: 1.0,

        lim: Limits::NONE 
    };
}