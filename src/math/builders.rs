use crate::units::*;

// Submodules
    mod hr;
    pub use hr::*;

    mod lr;
    pub use lr::*;
// 

// Basic builders
    /// - Very unsafe, just runs basic calculations
    #[derive(Clone, Debug)]
    pub struct TimeBuilder {
        pub alpha : Alpha,
        pub omega_0 : Omega,
        pub delta : Delta
    }

    impl TimeBuilder {
        #[inline]
        pub fn new(delta : Delta, omega_0 : Omega, alpha : Alpha) -> Self {
            Self { delta, omega_0, alpha }
        }
    }

    impl Iterator for TimeBuilder {
        type Item = Time;

        fn next(&mut self) -> Option<Self::Item> {
            // If there is a solution for the quadratic equation, the iterator will yield a value
            if let Some(time) = Time::positive_travel_time(self.delta, self.omega_0, self.alpha) {
                // Constant velocity, will be incremented
                self.omega_0 += self.alpha * time;
                Some(time)
            } else {
                None
            }
        }
    }

    /// - Will always succeed
    #[derive(Clone, Debug)]
    pub struct DeltaBuilder {
        pub alpha : Alpha,
        pub omega_0 : Omega,
        pub time : Time
    }

    impl Iterator for DeltaBuilder {
        type Item = Delta;

        fn next(&mut self) -> Option<Self::Item> {
            let delta = Delta::from_omega_alpha(self.omega_0, self.alpha, self.time);
            self.omega_0 += self.alpha * self.time;
            Some(delta)
        }
    }
// 

// Stepper motor builders
    /// Reasons why the iterators stopped yielding values
    #[derive(Default, Clone, Copy, Debug)]
    pub enum StopReason {
        #[default]
        None,
        
        // Operational
        MaxSpeed,
        MaxDelta,
        MaxSteps,

        // Error
        Overload,
        TravelProblems
    }   

// Advanced builders
    pub struct Path<D : Iterator<Item = [Delta; N]>, T : Iterator<Item = Time>, const N : usize> {
        iter_d : D,
        iter_t : T,

        stack_d : Vec<[Delta; N]>,
        stack_t : Vec<Time>,

        stack_size : usize
    }

    impl<D : Iterator<Item = [Delta; N]>, T : Iterator<Item = Time>, const N : usize> Path<D, T, N> {
        pub fn new(iter_d : D, iter_t : T, stack_size : usize) -> Self {
            Self {
                iter_d,
                iter_t,

                stack_d: vec![[Delta::ZERO; N]; stack_size],
                stack_t: vec![Time::ZERO; stack_size],

                stack_size
            }
        }

        #[inline]
        fn stack_d(&self, i : usize, n : usize) -> Delta {
            self.stack_d[i % self.stack_size][n]
        }

        #[inline]
        fn stack_t(&self, i : usize) -> Time {
            self.stack_t[i % self.stack_size]
        }
    }
//