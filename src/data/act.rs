use syunit::*;

use crate::ActuatorError;

/// Stores distance limits for the actuator
#[derive(Debug, Clone, Default)]
pub struct ActuatorLimits<U : UnitSet> {
    /// Limit for minimum position
    pub min : Option<U::Position>,  
    /// Limit for maximum position
    pub max : Option<U::Position>
}

/// The current state of the actuator, including some information and the underlying [ActuatorError] if there is one
#[derive(Clone, Debug, Default)]
pub enum ActuatorState<U : UnitSet> {
    /// The actuator is inactive
    #[default]
    Idle,
    /// The actuator is moving in the underlying [Direction]
    Moving(Direction),
    /// The acutator has ran into an [ActuatorError]
    Error(ActuatorError<U>)
}

/// Stores variables that can change in the process of the program
#[derive(Clone, Debug, Default)]
pub struct ActuatorVars<U : UnitSet> {
    /* State */
        /// The current state of the actuator
        pub state : ActuatorState<U>, 
        /// The current position of the actuator
        pub position : U::Position,
        /// The current velocity of the actuator if known, `None` otherwise
        pub velocity : Option<U::Velocity>,
        /// The current velocity of the actuator if known, `None` otherwise
        pub acceleration : Option<U::Acceleration>,
    /**/

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