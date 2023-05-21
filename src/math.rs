use crate::StepperConst;
use crate::data::{LinkedData, CompVars};
use crate::units::*;

// Submodules
// pub mod actors;

/// Methods for calculating stepper motor acceleration curves
pub mod curve;

/// Methods for calculating forces acting upon components and motors
pub mod force;
pub use force::{forces_joint, forces_segment};

/// Methods for calculating inertias of assemblies
pub mod inertia;

/// Methods for calculating paths and movements
pub mod path;
pub use path::{PathBuilder, PathNode};

/// Helper struct for creating stepper motor curves
#[derive(Debug, Clone)]
pub struct CurveBuilder<'a> {
    /// Acceleration of motor
    pub alpha : Alpha,
    /// Start velocity
    pub omega_0 : Omega,
    /// Motor speed
    pub omega : Omega,
    /// Current absolute position
    pub gamma : Gamma,
    /// Last delta distance traveled
    pub delta : Delta,

    /// Time traveled within the last node
    pub time : Time,

    consts : &'a StepperConst,
    var : &'a CompVars,
    lk : &'a LinkedData
}

impl<'a> CurveBuilder<'a> {
    /// Creates a new `CurveBuilder`
    pub fn new(consts : &'a StepperConst, var : &'a CompVars, lk : &'a LinkedData, omega_0 : Omega) -> Self {
        if !var.f_bend.is_normal() {
            panic!("Invaild bend factor! ({})", var.f_bend);
        }
    
        if !lk.s_f.is_normal() {
            panic!("Invalid safety factor! ({})", lk.s_f);
        }

        Self {
            alpha: Alpha::ZERO,
            omega_0: omega_0,
            omega: omega_0,
            gamma: Gamma::ZERO,
            delta: Delta::ZERO,

            time: Time::ZERO,

            consts,
            var,
            lk
        }
    }

    /// Resets the builder
    #[inline]
    pub fn reset(&mut self) {
        self.alpha = Alpha::ZERO;
        self.omega_0 = Omega::ZERO;
        self.gamma = Gamma::ZERO;
        self.delta = Delta::ZERO;
        self.time = Time::ZERO;
    }

    /// Set the speed of this `CurveBuilder` 
    #[inline(always)]
    pub fn set_speed(&mut self, omega : Omega) {
        self.omega = omega;
    }

    /// Max speed that the curve will be generated for
    #[inline(always)]
    pub fn max_speed(&self) -> Omega {
        self.consts.max_speed(self.lk.u)
    }

    /// Checks whether the motor accelerated when travelling the last node
    pub fn was_accel(&self) -> bool {
        self.omega > self.omega_0
    }

    /// Checks whether the motor deccelerated when travelling the last node
    pub fn was_deccel(&self) -> bool {
        self.omega < self.omega_0
    }

    /// Generate the next node
    pub fn next(&mut self, mut delta : Delta, mut omega_tar : Omega) -> (Time, f32) {
        if (omega_tar.0 * self.omega.0) < 0.0 {
            omega_tar = Omega::ZERO;
        } else if omega_tar < Omega::ZERO {
            delta = -delta;
        }

        self.alpha = self.consts.alpha_max_dyn(
            force::torque_dyn(self.consts, 
                    (self.omega /* + omega_tar */).abs() /* / 2.0 */ / self.var.f_bend, self.lk.u) / self.lk.s_f * self.var.f_bend, 
                self.var
        ).unwrap();

        let alpha_max = Alpha((omega_tar.powi(2) - self.omega.powi(2)).0 / 2.0 / delta.0).abs();

        if alpha_max < self.alpha {
            self.alpha = alpha_max;
        }
        
        if omega_tar < self.omega {
            self.alpha = -self.alpha;
        }

        let time; let omega;
        if self.alpha != Alpha::ZERO {
            ( time, omega ) = curve::next_node_simple(delta, self.omega, self.alpha);

            if omega.is_nan() {
                dbg!(time, self.alpha, self.omega, omega_tar, delta);
                dbg!((self.omega / self.alpha).0.powi(2) + (2.0 * delta.0 / self.alpha.0));
            }
        } else {
            omega = omega_tar;
            time = delta / omega_tar;
        }

        self.gamma += delta;
        self.delta = delta;

        self.omega_0 = self.omega;
        self.omega = omega;
        self.time = time;

        let mut f = (self.omega - self.omega_0) / (omega_tar - self.omega_0);

        if self.alpha == Alpha::ZERO {
            f = f32::INFINITY;
        }

        if f > 1.0 {
            self.omega = omega_tar;
            self.time = 2.0 * delta / (self.omega + self.omega_0);
        }

        ( self.time, f )
    }

    /// Calculates the next step in an acceleration curve and returns the step-time
    pub fn next_step_pos(&mut self) -> Time {
        self.next(self.consts.step_ang(), self.max_speed()).0
    }

    /// Calculates the next step in an decceleration curve and returns the step-time
    pub fn next_step_neg(&mut self) -> Time {
        self.next(self.consts.step_ang(), -self.max_speed()).0
    }

    #[inline(always)]
    fn next_step(&mut self, omega : Omega, time : &mut Time) -> bool {
        if omega > self.omega {
            *time = self.next_step_pos();

            self.omega >= omega 
        } else {
            *time = self.next_step_neg();

            self.omega <= omega              
        }
    }

    /// Creates the curve required to drive from the current curve speed to the given speed `omega`
    /// 
    /// # Errors
    /// 
    /// Returns an error if the given `omega` is higher than the `max_speed()` of the `CurveBuilder` 
    pub fn to_speed(&mut self, omega : Omega) -> Result<Vec<Time>, crate::Error> {
        if omega.abs() > self.max_speed() {
            return Err(crate::lib_error(format!("The given omega is too high! {}", omega)))
        }

        let mut curve = vec![];

        loop {
            let mut time : Time = Time::ZERO;
            if self.next_step(omega, &mut time) {
                break;
            }
            curve.push(time);
        }

        Ok(curve)
    }

    /// Creates the curve required to drive from the current curve speed to the given 
    /// speed `omega` and writes it directly to the given buffer `curve`
    /// 
    /// # Errors
    /// 
    /// Returns an error if the given `omega` is higher than the `max_speed()` of the `CurveBuilder` 
    pub fn to_speed_buffer(&mut self, curve : &mut [Time], omega : Omega) -> Result<(), crate::Error> {
        if omega > self.max_speed() {
            return Err(crate::lib_error(format!("The given omega is too high! {}", omega)))
        }

        for elem in curve {
            if self.next_step(omega, elem) {
                break;
            }
        }

        Ok(())
    }

    /// Return a pathnode stored in the builder
    pub fn get_node(&self) -> path::PathNode {
        path::PathNode {
            delta: self.delta,
            omega_0: self.omega_0
        }
    }

    /// Load a node into the builder
    pub fn load_node(&mut self, node : &path::PathNode) {
        self.delta = node.delta;
        self.omega = node.omega_0;
    } 
}