use syunit::*;

/// Stores distance limits for the component
#[derive(Debug, Clone, Default)]
pub struct Limits<U : UnitSet> {
    /// Limit for minimum pos distance
    pub min : Option<U::Position>,  
    /// Limit for maximum pos distance
    pub max : Option<U::Position>
}

impl<U : UnitSet> Limits<U> {
    /// No limits set
    const NONE : Self = Self { min: None, max: None };
}

/// Stores variables that can change in the process of the program
#[derive(Clone, Debug, Default)]
pub struct ActuatorVars<U : UnitSet = Rotary> {
    /// Load torque general in both directions (e.g. friction), unit Nm
    pub force_load_gen : U::Force,

    /// Load torque directionally dependent, unit Nm
    pub force_load_dir : U::Force,

    /// Load inertia, unit kgm^2
    pub inertia_load : U::Inertia,

    /// Limits of the component
    pub lim : Limits<U>
}

impl<U : UnitSet> ActuatorVars<U>  {
    /// Zero value component data, used for initialization
    pub const ZERO : Self = Self { 
        force_load_gen: U::Force::ZERO, 

        force_load_dir: U::Force::ZERO,

        inertia_load: U::Inertia::ZERO, 

        lim: Limits::NONE 
    };

    /// Returns `None` if there is an overlaod
    pub fn force_after_load(&self, force : U::Force, direction : Direction) -> Option<U::Force> {
        let mut force = force - self.force_load_gen;
        
        if direction.as_bool() {
            force -= self.force_load_dir;
        } else {
            force += self.force_load_dir;
        }

        if force > U::Force::ZERO {
            Some(force)
        } else {
            None
        }
    }

    /// Returns the smaller force of the two directions after applying both loads
    pub fn force_after_load_lower(&self, force : U::Force) -> Option<U::Force> {
        let force = force - self.force_load_gen - self.force_load_dir.abs();
        
        if force > U::Force::ZERO {
            Some(force)
        } else {
            None
        }
    }

    /// Returns the given inertia after applying the load inertia to it
    #[inline]
    pub fn inertia_after_load(&self, inertia : U::Inertia) -> U::Inertia {
        inertia + self.inertia_load
    }
}