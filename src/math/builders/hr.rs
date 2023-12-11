use crate::StepperConst;
use crate::act::StepperMotor;
use crate::data::{StepperConfig, ActuatorVars, MicroSteps};

use super::*;

/// - Considers loads, changing torque etc.
/// - Accelerate and deccelerate
/// - Only positive deltas
#[derive(Clone, Debug)]
pub struct HRStepBuilder {
    pub delta : Delta,
    pub omega_0 : Omega,
    pub alpha : Alpha,

    pub consts : StepperConst,
    pub vars : ActuatorVars,
    pub config : StepperConfig,

    pub deccel : bool,
    pub omega_max : Omega,

    // Errros
    pub reason : StopReason
}

impl HRStepBuilder {
    pub fn new(omega_0 : Omega, consts : StepperConst, vars : ActuatorVars, config : StepperConfig, omega_max : Omega, micro : MicroSteps) -> Self {
        Self {
            delta: consts.step_angle(micro),
            omega_0: omega_0.abs(),
            alpha: Alpha::ZERO,

            consts, 
            vars,
            config,

            deccel: false,
            omega_max,

            reason: StopReason::None
        }
    }

    pub fn from_motor<M : StepperMotor + ?Sized>(motor : &M, omega_0 : Omega) -> Self {
        Self::new(omega_0, motor.consts().clone(), motor.vars().clone(), motor.config().clone(), motor.omega_max(), motor.microsteps())
    }

    pub fn start_accel(&mut self) {
        self.deccel = false;
    }

    pub fn start_deccel(&mut self) {
        self.deccel = true;
    }
}

impl Into<TimeBuilder> for HRStepBuilder {
    fn into(self) -> TimeBuilder {
        TimeBuilder {
            delta: self.delta,
            omega_0: self.omega_0,
            alpha: self.alpha
        }
    }
}

impl Iterator for HRStepBuilder {
    type Item = Time;

    fn next(&mut self) -> Option<Self::Item> {
        // Max speed reached
        if self.omega_0.abs() >= self.omega_max.abs() {
            self.reason = StopReason::MaxSpeed;
            return None;
        }

        // Apply new alpha for speed
        if let Some(alpha) = self.consts.alpha_max_for_omega(     // TODO: Rework builders
            &self.vars, &self.config, self.omega_0, sylo::Direction::CW
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
pub struct HRCtrlStepBuilder {
    builder : HRStepBuilder,
    omega_tar : Omega,
    speed_f : f32
}

impl HRCtrlStepBuilder {
    pub fn new(omega_0 : Omega, consts : StepperConst, vars : ActuatorVars, data : StepperConfig, 
    omega_max : Omega, micro : MicroSteps) -> Self {
        Self::from_builder(
            HRStepBuilder::new(omega_0, consts, vars, data, omega_max, micro)
        )
    }

    pub fn from_builder(builder : HRStepBuilder) -> Self {
        Self {
            builder,
            omega_tar: Omega::ZERO,
            speed_f: 1.0
        }
    }

    // Accel / Deccel 
        #[inline]
        pub fn is_accel(&self) -> bool {
            !self.builder.deccel
        }

        #[inline]
        pub fn is_deccel(&self) -> bool {
            self.builder.deccel
        }
    // 

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

    pub fn set_speed_f(&mut self, speed_f : f32) {
        if (speed_f > 1.0) | (speed_f < 0.0) {
            panic!("Bad speed factor! {}", speed_f);
        }

        self.speed_f = speed_f;
    }

    pub fn override_delta(&mut self, delta : Delta) {
        self.builder.delta = delta
    }

    pub fn stop_reason(&self) -> StopReason {
        self.builder.reason
    }

    pub fn t_tar(&self) -> Time {
        self.builder.delta / self.omega_tar / self.speed_f
    }
}

impl Iterator for HRCtrlStepBuilder {
    type Item = Time;

    fn next(&mut self) -> Option<Self::Item> {
        // Check if the curve is done
        if (self.builder.deccel & (self.builder.omega_0 <= self.omega_tar)) |       // Deccelerating and speed is below target
            (!self.builder.deccel & (self.builder.omega_0 >= self.omega_tar)) {     // Accelerating and speed is above target
            self.builder.reason = StopReason::MaxSpeed;
            return None;
        }

        self.builder.next().map(|t| t / self.speed_f)
    }
}

#[derive(Clone, Debug)]
pub struct HRLimitedStepBuilder {
    builder : HRCtrlStepBuilder,
    steps_max : i64,
    steps_curr : i64,

    last_val : Time,
    reach_dist : Option<i64>
}

impl HRLimitedStepBuilder {
    pub fn new(omega_0 : Omega, consts : StepperConst, vars : ActuatorVars, data : StepperConfig, 
    omega_max : Omega, micro : MicroSteps) -> Self {
        Self {
            builder: HRCtrlStepBuilder::new(omega_0, consts, vars, data, omega_max, micro),
            steps_max: 0,
            steps_curr: 0,

            last_val: Time::ZERO,
            reach_dist: None
        }
    }

    pub fn from_builder(builder : HRStepBuilder) -> Self {
        Self {
            builder: HRCtrlStepBuilder::from_builder(builder),
            steps_max: 0,
            steps_curr: 0,

            last_val: Time::ZERO,
            reach_dist: None
        }
    }

    pub fn set_omega_tar(&mut self, omega_tar : Omega) -> Result<(), crate::Error> {
        self.builder.set_omega_tar(omega_tar)
    }

    pub fn set_speed_f(&mut self, speed_f : f32) {
        self.builder.set_speed_f(speed_f)
    }

    pub fn set_steps_max(&mut self, steps_max : u64) {
        self.steps_max = steps_max as i64;
    }

    pub fn stop_reason(&self) -> StopReason {
        self.builder.stop_reason()
    }
}

impl Iterator for HRLimitedStepBuilder {
    type Item = Time;

    fn next(&mut self) -> Option<Self::Item> {
        self.steps_curr += 1;

        if self.steps_curr > self.steps_max {
            self.builder.builder.reason = StopReason::MaxSteps;
            return None;
        }

        if self.steps_max != 1 {
            if self.steps_curr > (self.steps_max / 2) {
                if !self.builder.builder.deccel {
                    self.set_omega_tar(Omega::ZERO).unwrap(); 
                    
                    if (self.steps_max % 2) == 1 {
                        return Some(self.last_val)
                    }
                }

                if let Some(dist) = self.reach_dist {
                    if (self.steps_max / 2 - dist) >= (self.steps_curr - self.steps_max / 2) {
                        return Some(self.last_val)
                    }
                }
            }
        }

        if let Some(val) = self.builder.next() {
            self.last_val = val;
            Some(val)
        } else {
            if self.reach_dist.is_none() {
                self.reach_dist = Some(self.steps_curr);
                self.last_val = self.builder.t_tar();
            }

            Some(self.last_val)
        }
    }
}