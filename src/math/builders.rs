use crate::units::*;

// Submodules
    mod hr;
    pub use hr::*;
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
    pub struct Path<const N : usize> {
        builders : [HRCtrlStepBuilder; N],
        stack_d : Vec<[Delta; N]>,
        stack_o : Vec<[Omega; N]>,
        stack_t : Vec<Time>,

        stack_size : usize
    }

    impl<const N : usize> Path<N> {
        pub fn new(builders : [HRCtrlStepBuilder; N], stack_size : usize) -> Self {
            Self {
                builders,

                stack_d: vec![[Delta::ZERO; N]; stack_size],
                stack_o: vec![[Omega::ZERO; N]; stack_size],
                stack_t: vec![Time::ZERO; stack_size],

                stack_size
            }
        }

        // Stack writing
            #[inline]
            pub fn index_d(&self, i : usize, n : usize) -> Delta {
                self.stack_d[i % self.stack_size][n]
            }   

            #[inline]
            pub fn index_o(&self, i : usize, n : usize) -> Omega {
                self.stack_o[i & self.stack_size][n]
            }

            #[inline]
            pub fn index_t(&self, i : usize) -> Time {
                self.stack_t[i % self.stack_size]
            }

            pub fn write_t(&mut self, i : usize, t : Time) {
                self.stack_t[i] = t
            }

            pub fn write_o(&mut self, i : usize, n : usize, o : Omega) {
                self.stack_o[i][n] = o;
            }
        // 

        #[inline]
        pub fn next_omega(&self, i : usize, n : usize) -> Omega {
            Omega::from_delta_time_0(self.index_o(i, n), self.index_d(i, n), self.index_t(i))
        }

        #[inline]
        pub fn omega_max(&self, i : usize, n : usize) -> Omega {
            self.index_d(i, n) / self.index_t(i)
        }

        #[inline]
        pub fn omega_diff_req(&self, i : usize, n : usize) -> Omega {
            self.omega_max(i, n) - self.index_o(i, n)
        }

        // pub fn next(&mut self, i : usize, n : usize) {
        //     // let o_aw = 
        //     // self.builders[n].override_delta(self.index_d(i, n));
        //     // self.builders[n].set_omega_tar(self.index_o(i + 1, n)).unwrap();      // TODO: Handle overspeed

        //     // if let Some(time) = self.builders[n].next() {
        //     //     if time > self.index_t(i) {
        //     //         self.write_t(i, time);

        //     //         for _n in 0 .. n {
                        
        //     //         }
        //     //     }
        //     // } 
        // }
    }
//