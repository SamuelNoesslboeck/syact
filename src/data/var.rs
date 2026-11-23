use syunit::*;

/// Stores distance limits for the actuator
#[derive(Debug, Clone, Default)]
pub struct ActuatorLimits<U : UnitSet> {
    /// Limit for minimum position
    pub min : Option<U::Position>,  
    /// Limit for maximum position
    pub max : Option<U::Position>
}

impl<U : UnitSet> ActuatorLimits<U> {
    /// No limits set yet, both are `None`
    const NONE : Self = Self { min: None, max: None };
}

/// Stores variables that can change in the process of the program
#[derive(Clone, Debug, Default)]
pub struct ActuatorVars<U : UnitSet = Rotary> {
    /* Load data */
        /// Load torque general in both directions (e.g. friction)
        pub load_force_gen : U::Force,
        /// Load torque directionally dependent
        pub load_force_dir : U::Force,
        /// Load inertia, 
        pub load_inertia : U::Inertia,
    /**/

    /* Limits */
        /// Positional limits of the actuator
        pub pos_limits : ActuatorLimits<U>,
        /// Maximum velocity of the actuator, `None` means nothing has been set yet
        pub velocity_max : Option<U::Velocity>,
        /// Maximum acceleration of the actuator, `None` means nothing has been set yet
        pub acceleration_max : Option<U::Acceleration>,
        /// Maximum jolt of the actuator, `None` means nothing has been set yet
        pub jolt_max : Option<U::Jolt>
    /**/
}

impl<U : UnitSet> ActuatorVars<U>  {
    /// Zero value component data, used for initialization
    pub const ZERO : Self = Self { 
        load_force_gen: U::Force::ZERO, 
        load_force_dir: U::Force::ZERO,
        load_inertia: U::Inertia::ZERO, 

        pos_limits: ActuatorLimits::NONE,
        velocity_max: None,
        acceleration_max: None,
        jolt_max: None
    };

    /// Returns `None` if there is an overlaod
    pub fn force_after_load(&self, force : U::Force, direction : Direction) -> Option<U::Force> {
        let mut force = force - self.load_force_gen;
        
        if direction.as_bool() {
            force -= self.load_force_dir;
        } else {
            force += self.load_force_dir;
        }

        if force > U::Force::ZERO {
            Some(force)
        } else {
            None
        }
    }

    /// Returns the smaller force of the two directions after applying both loads
    pub fn force_after_load_lower(&self, force : U::Force) -> Option<U::Force> {
        let force = force - self.load_force_gen - self.load_force_dir.abs();
        
        if force > U::Force::ZERO {
            Some(force)
        } else {
            None
        }
    }

    /// Returns the given inertia after applying the load inertia to it
    #[inline]
    pub fn inertia_after_load(&self, inertia : U::Inertia) -> U::Inertia {
        inertia + self.load_inertia
    }
}