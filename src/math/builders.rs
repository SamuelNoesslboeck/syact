use crate::StepperConst;
use crate::data::{CompData, CompVars};
use crate::units::*;

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
            if let Some(time) = Time::positive_travel_time(self.delta, self.omega_0, self.alpha) {
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
    #[derive(Default, Clone, Copy, Debug)]
    pub enum StopReason {
        #[default]
        None,
        
        // Operational
        MaxSpeed,

        // Error
        Overload,
        TravelProblems
    }   

    /// - Considers loads, changing torque etc.
    /// - Accelerate and deccelerate
    /// - Only positive deltas
    #[derive(Clone, Debug)]
    pub struct StepTimeBuilder {
        pub delta : Delta,
        pub omega_0 : Omega,
        pub alpha : Alpha,

        pub consts : StepperConst,
        pub vars : CompVars,
        pub data : CompData,

        pub deccel : bool,
        pub omega_max : Omega,

        // Errros
        pub reason : StopReason
    }

    impl StepTimeBuilder {
        pub fn new(omega_0 : Omega, consts : StepperConst, vars : CompVars, data : CompData, omega_max : Omega, micro : u8) -> Self {
            Self {
                delta: consts.step_ang(micro),
                omega_0: omega_0.abs(),
                alpha: Alpha::ZERO,

                consts, 
                vars,
                data,

                deccel: false,
                omega_max,

                reason: StopReason::None
            }
        }

        pub fn start_accel(&mut self) {
            self.deccel = false;
        }

        pub fn start_deccel(&mut self) {
            self.deccel = true;
        }
    }

    impl Into<TimeBuilder> for StepTimeBuilder {
        fn into(self) -> TimeBuilder {
            TimeBuilder {
                delta: self.delta,
                omega_0: self.omega_0,
                alpha: self.alpha
            }
        }
    }

    impl Iterator for StepTimeBuilder {
        type Item = Time;

        fn next(&mut self) -> Option<Self::Item> {
            if self.omega_0.abs() >= self.omega_max.abs() {
                self.reason = StopReason::MaxSpeed;
                return None;
            }

            // Apply new alpha for speed
            if let Ok(alpha) = self.consts.alpha_max_dyn(
                crate::math::force::torque_dyn(&self.consts, self.omega_0, self.data.u) / self.data.s_f, &self.vars
            ) {
                self.alpha = alpha;     // Update the alpha

                // `torque_dyn` will always be positive, so negate it if the motor is deccelerating
                if self.deccel {
                    self.alpha = -self.alpha;
                }
            } else {
                self.reason = StopReason::Overload;         // The motor is overloaded
                return None;
            }

            // Check if a valid travel time can be generated
            if let Some(mut time) = Time::positive_travel_time(self.delta, self.omega_0, self.alpha) {
                self.omega_0 += self.alpha * time;

                // Check if the omega would overextend
                if self.omega_0.abs() > self.omega_max.abs() {
                    // Correct omega and step time
                    time = 2.0 * self.delta / (self.omega_max + self.omega_0 - self.alpha * time);
                    self.omega_0 = self.omega_max;

                    // After this block, the next iteration will return None
                }

                Some(time)
            } else {
                self.reason = StopReason::TravelProblems;     // Iterator stopped because of travel calculation issues
                None
            }
        }
    }

    #[derive(Clone, Debug)]
    pub struct CtrlStepTimeBuilder {
        builder : StepTimeBuilder,
        omega_tar : Omega
    }
    
    impl CtrlStepTimeBuilder {
        pub fn new(omega_0 : Omega, consts : StepperConst, vars : CompVars, data : CompData, 
        omega_max : Omega, micro : u8) -> Self {
            Self {
                builder: StepTimeBuilder::new(omega_0, consts, vars, data, omega_max, micro),
                omega_tar: Omega::ZERO
            }
        }

        pub fn from_builder(builder : StepTimeBuilder) -> Self {
            Self {
                builder,
                omega_tar: Omega::ZERO
            }
        }

        pub fn set_omega_tar(&mut self, omega_tar : Omega) -> Result<(), crate::Error> {
            if omega_tar.abs() > self.builder.omega_max.abs() {
                return Err("Target omega is above max_speed!".into())
            }

            self.omega_tar = omega_tar;

            // Switch acceleration mode
            if self.omega_tar > self.builder.omega_0 {
                self.builder.start_accel()
            } else {
                self.builder.start_deccel()
            }

            Ok(())
        }

        pub fn stop_reason(&self) -> StopReason {
            self.builder.reason
        }
    }

    impl AsRef<StepTimeBuilder> for CtrlStepTimeBuilder {
        fn as_ref(&self) -> &StepTimeBuilder {
            &self.builder
        }
    }

    impl AsMut<StepTimeBuilder> for CtrlStepTimeBuilder {
        fn as_mut(&mut self) -> &mut StepTimeBuilder {
            &mut self.builder
        }
    }

    impl Iterator for CtrlStepTimeBuilder {
        type Item = Time;

        fn next(&mut self) -> Option<Self::Item> {
            // Check if the curve is done
            if (self.builder.deccel & (self.builder.omega_0 <= self.omega_tar)) |       // Deccelerating and speed is below target
                (!self.builder.deccel & (self.builder.omega_0 >= self.omega_tar)) {     // Accelerating and speed is above target
                self.builder.reason = StopReason::MaxSpeed;
                return None;
            }

            self.builder.next()
        }
    }
// 

// Advanced builders 

// 