use crate::{SyncActuator, Setup, SpeedFactor};
use crate::act::parent::{ActuatorParent, RatioActuatorParent};
use crate::units::*;

pub struct StateActuator<A : SyncActuator, const C : usize> {
    ctrl : A,
    states : [Gamma; C]
}

impl<A : SyncActuator, const C : usize> StateActuator<A, C> {
    pub fn new(ctrl : A, states : [Gamma; C]) -> Self {
        Self {
            ctrl,
            states
        }
    }

    #[inline]
    pub fn get_state(&self, index : usize) -> Gamma {
        self.states[index]
    }

    #[inline]
    pub fn set_state(&mut self, index : usize, gamma : Gamma) {
        self.states[index] = gamma;
    }

    // Driving
        pub fn drive_to_state(&mut self, index : usize, speed : SpeedFactor) -> Result<(), crate::Error> {
            self.drive_abs(self.get_state(index), speed)
        }

        pub fn drive_to_state_async(&mut self, index : usize, speed : SpeedFactor) -> Result<(), crate::Error> {
            self.drive_abs_async(self.get_state(index), speed)
        }
    // 
}

// Parent
    impl<A : SyncActuator, const C : usize> Setup for StateActuator<A, C> {
        fn setup(&mut self) -> Result<(), crate::Error> {
            self.ctrl.setup() 
        }
    }

    impl<A : SyncActuator, const C : usize> ActuatorParent for StateActuator<A, C> {
        type Child = A;

        #[inline]
        fn child(&self) -> &Self::Child {
            &self.ctrl
        }

        #[inline]
        fn child_mut(&mut self) -> &mut Self::Child {
            &mut self.ctrl
        }
    }

    impl<A : SyncActuator, const C : usize> RatioActuatorParent for StateActuator<A, C> {
        #[inline]
        fn ratio(&self) -> f32 {
            1.0
        }
    }
// 