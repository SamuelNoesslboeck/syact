use crate::StepperConst;
use crate::data::{CompData, CompVars};
use crate::math::force::torque_dyn;
use crate::prelude::StepperMotor;

use super::*;

#[derive(Clone, Debug)]
pub struct LRStepBuilder {
    pub delta : Delta,
    pub omega_0 : Omega, 
    pub alpha : Alpha,

    pub omega_max : Omega
}

impl LRStepBuilder {
    pub fn new(omega_0 : Omega, consts : &StepperConst, vars : &CompVars, data : &CompData, omega_max : Omega, micro : u8) -> Self {
        Self {
            delta: consts.step_ang(micro),
            omega_0,
            alpha: consts.alpha_max_dyn(
                torque_dyn(consts, consts.omega_max(data.u), data.u, 0.0), vars
            ).unwrap(),     // TODO: Handle overload
            omega_max
        }
    }

    pub fn from_motor<M : StepperMotor>(motor : &M, omega_0 : Omega, omega_max : Omega) -> Self {
        Self::new(omega_0, motor.consts(), motor.vars(), motor.data(), omega_max, motor.micro())
    }

    pub fn t_accel(&self) -> Time {
        (self.omega_max - self.omega_0) / self.alpha
    }

    pub fn delta_accel(&self) -> Delta {
        Delta::from_omega_start_end(self.omega_0, self.omega_max, self.t_accel())
    }
}

impl Iterator for LRStepBuilder {
    type Item = Time;

    fn next(&mut self) -> Option<Self::Item> {
        if self.omega_0 > self.omega_max {
            return None;
        }

        let time = Time::positive_travel_time(self.delta, self.omega_0, self.alpha).unwrap();   // TODO: Handle None
        self.omega_0 += self.alpha * time;
        Some(time)
    }
}

#[derive(Clone, Debug)]
pub struct LRCtrlStepBuilder {
    builder : HRStepBuilder,
    omega_tar : Omega,
    speed_f : f32
}

// WIP