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
pub struct ActuatorVars {
    /// Load torque general in both directions (e.g. friction), unit Nm
    pub force_load_gen : Force,

    /// Load torque directionally dependent, unit Nm
    pub force_load_dir : Force,

    /// Load inertia, unit kgm^2
    pub inertia_load : Inertia,

    /// Limits of the component
    pub lim : Limits
}

impl ActuatorVars  {
    /// Zero value component data, used for initialization
    pub const ZERO : Self = Self { 
        force_load_gen: Force::ZERO, 

        force_load_dir: Force::ZERO,

        inertia_load: Inertia::ZERO, 

        lim: Limits::NONE 
    };
}