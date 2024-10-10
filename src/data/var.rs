use syunit::*;

/// Stores distance limits for the component
#[derive(Debug, Clone, Default)]
pub struct Limits {
    /// Limit for minimum gamma distance
    pub min : Option<AbsPos>,  
    /// Limit for maximum gamma distance
    pub max : Option<AbsPos>
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

    /// Returns `None` if there is an overlaod
    pub fn force_after_load(&self, force : Force, direction : Direction) -> Option<Force> {
        let mut force = force - self.force_load_gen;
        
        if direction.as_bool() {
            force -= self.force_load_dir;
        } else {
            force += self.force_load_dir;
        }

        if force > Force::ZERO {
            Some(force)
        } else {
            None
        }
    }

    /// Returns the smaller force of the two directions after applying both loads
    pub fn force_after_load_lower(&self, force : Force) -> Option<Force> {
        let force = force - self.force_load_gen - self.force_load_dir.abs();
        
        if force > Force::ZERO {
            Some(force)
        } else {
            None
        }
    }

    /// Returns the given inertia after applying the load inertia to it
    #[inline]
    pub fn inertia_after_load(&self, inertia : Inertia) -> Inertia {
        inertia + self.inertia_load
    }
}