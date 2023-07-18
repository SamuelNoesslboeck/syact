use crate::units::*;

/// Stores distance limits for the component
#[derive(Debug, Clone, Default)]
pub struct Limits {
    /// Limit for minimum gamma distance
    pub min : Option<Gamma>,  
    /// Limit for maximum gamma distance
    pub max : Option<Gamma>
}

impl Limits {
    /// No limits set
    const NONE : Self = Self { min: None, max: None };
}

/// Stores variables that can change in the process of the program
#[derive(Clone, Debug, Default)]
pub struct CompVars {
    /// Load torque, unit Nm
    pub t_load : Force,
    /// Load inertia, unit kgm^2
    pub j_load : Inertia,

    /// Bend factor for stepper 
    pub bend_f : f32,

    /// Limits of the component
    pub lim : Limits
}

impl CompVars  {
    /// Zero value component data, used for initialization
    pub const ZERO : Self = Self { 
        t_load: Force::ZERO, 
        j_load: Inertia::ZERO, 

        bend_f: 1.0,

        lim: Limits::NONE 
    };
}