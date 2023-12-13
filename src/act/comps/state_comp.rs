use crate::{SyncActuator, Setup, SpeedFactor};
use crate::act::parent::{ActuatorParent, RatioActuatorParent};
use crate::units::*;

pub struct StateActuator<A : SyncActuator, const C : usize> {
    ctrl : A,
    states : [Gamma; C],
    state : usize
}

impl<A : SyncActuator, const C : usize> StateActuator<A, C> {
    pub fn new(ctrl : A, states : [Gamma; C]) -> Self {
        Self {
            ctrl,
            states,
            state: 0
        }
    }

    #[inline]
    pub fn get_state_gamma(&self, index : usize) -> Gamma {
        self.states[index]
    }

    #[inline]
    pub fn set_state_gamma(&mut self, index : usize, gamma : Gamma) {
        self.states[index] = gamma;
    }

    #[inline]
    pub fn get_state(&self) -> usize {
        self.state
    }

    #[inline]
    pub fn set_state(&mut self, index : usize) {
        if index > C {
            panic!("State index out of bounds! (Index given: {}, Max: {})", index, C);
        }

        self.set_gamma(self.get_state_gamma(index))
    }

    // Driving
        pub fn drive_to_state(&mut self, index : usize, speed : SpeedFactor) -> Result<(), crate::Error> {
            self.drive_abs(self.get_state_gamma(index), speed).map(|ok| {
                self.state = index;
                ok
            })
        }

        pub fn drive_to_state_async(&mut self, index : usize, speed : SpeedFactor) -> Result<(), crate::Error> {
            self.drive_abs_async(self.get_state_gamma(index), speed).map(|ok| {
                self.state = index;
                ok
            })
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