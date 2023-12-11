use sylo::Direction;

use crate::{StepperConst, ActuatorVars, StepperConfig};
use crate::act::stepper::{StepError, Controller};
use crate::data::{SpeedFactor, MicroSteps};
use crate::math;
use crate::units::*;

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub enum DriveMode {
    ConstOmega(Omega, Direction),
    ConstFactor(SpeedFactor, Direction),
    FixedDistance(Delta, Omega, SpeedFactor),
    Stop,
    Inactive
}

#[derive(Clone, Debug)]
pub enum DriveError {
    OmegaMaxTooHigh,
    OmegaOutTooHigh,
    Overload,
    Step(StepError)
}

impl std::fmt::Display for DriveError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_fmt(format_args!("{:?}", self))
    }
}

impl std::error::Error for DriveError { }

pub trait StepperBuilder : Iterator<Item = Time> {
    // General constructor
        fn new(consts : StepperConst) -> Self
        where 
            Self: Sized;
    // 

    // Data
        fn consts(&self) -> &StepperConst;

        fn vars(&self) -> &ActuatorVars;

        fn config(&self) -> &StepperConfig;

        fn set_config(&mut self, config : StepperConfig);

        fn microsteps(&self) -> MicroSteps;

        fn set_microsteps(&mut self, microsteps : MicroSteps);

        fn step_angle(&self) -> Delta;
    //

    // Loads
        fn apply_gen_force(&mut self, force : Force) -> Result<(), DriveError>;

        /// Value positive in CW direction
        fn apply_dir_force(&mut self, force : Force) -> Result<(), DriveError>;

        fn apply_inertia(&mut self, inertia : Inertia);
    //

    // Omega max
        fn omega_max(&self) -> Omega;

        fn set_omega_max(&mut self, omega : Omega) -> Result<(), DriveError>;
    // 

    // Regulation
        fn drive_mode(&self) -> &DriveMode;

        fn set_drive_mode<C : Controller>(&mut self, mode : DriveMode, ctrl : &mut C) -> Result<(), DriveError>;
    //   
}

// ################
// #    COMMON    #
// ################
    pub struct StartStopBuilder {
        consts : StepperConst,
        vars : ActuatorVars,
        config : StepperConfig,

        // Speeds
        omega_start_stop : Omega,
        _omega_max : Option<Omega>,

        // Cache
        microsteps : MicroSteps,
        mode : DriveMode,
        _step_angle : Delta, 

        distance : u64,
        distance_counter : u64
    }

    impl StartStopBuilder {
        pub fn update_start_stop(&mut self) -> Result<(), DriveError> {
            self.omega_start_stop = math::kin::start_stop(
                self.vars.force_after_load_lower(self.consts.torque_stall).ok_or(DriveError::Overload)?, 
                self.vars.inertia_after_load(self.consts.inertia_motor), 
                self.consts.number_steps
            );

            // Reset omega max if it is too high
            if let Some(omega_max) = self._omega_max {
                if omega_max >= self.omega_start_stop {
                    self._omega_max = None;
                }
            }

            Ok(())
        }
    }

    impl Iterator for StartStopBuilder {
        type Item = Time;

        fn next(&mut self) -> Option<Self::Item> {
            match self.mode {
                DriveMode::ConstOmega(omega, _) => Some(omega),
                DriveMode::ConstFactor(factor, _) => Some(self.omega_max() * factor),
                DriveMode::FixedDistance(_, _, factor) => {
                    self.distance_counter += 1;

                    if self.distance_counter > self.distance {
                        self.mode = DriveMode::Inactive;
                        None
                    } else {
                        Some(self.omega_max() * factor)
                    }
                },
                DriveMode::Stop => {
                    self.mode = DriveMode::Inactive;
                    None
                },
                DriveMode::Inactive => None
            }.map(|omega| self.consts.step_time(omega, self.microsteps))
        }
    }

    impl StepperBuilder for StartStopBuilder {
        // General constructors
            fn new(consts : StepperConst) -> Self
            where 
                Self: Sized 
            {
                let mut _self = Self {
                    vars: ActuatorVars::ZERO,
                    config: StepperConfig::GEN,

                    omega_start_stop: Omega::INFINITY,
                    _omega_max: None,
                    microsteps: MicroSteps::default(),

                    distance: 0,
                    distance_counter: 0,
                    _step_angle: consts.step_angle(MicroSteps::default()),

                    mode: DriveMode::Stop,

                    consts
                };

                // TODO: Enable error
                _self.update_start_stop().unwrap();

                _self
            }
        // 

        // Data
            fn consts(&self) -> &StepperConst {
                &self.consts
            }

            fn vars(&self) -> &ActuatorVars {
                &self.vars
            }

            fn config(&self) -> &StepperConfig {
                &self.config
            }
            
            fn set_config(&mut self, config : StepperConfig) {
                self.config = config;
            }

            fn microsteps(&self) -> MicroSteps {
                self.microsteps
            }

            fn set_microsteps(&mut self, microsteps : MicroSteps) {
                self._step_angle = self.consts.step_angle(microsteps);
                self.microsteps = microsteps;
            }

            fn step_angle(&self) -> Delta {
                self._step_angle
            }
        // 

        // Loads
            fn apply_gen_force(&mut self, force : Force) -> Result<(), DriveError> {
                self.vars.force_load_gen = force;
                self.update_start_stop()
            }

            fn apply_dir_force(&mut self, force : Force) -> Result<(), DriveError> {
                self.vars.force_load_dir = force;
                self.update_start_stop()
            }
            
            fn apply_inertia(&mut self, inertia : Inertia) {
                self.vars.inertia_load = inertia;
                self.update_start_stop().unwrap();      // Save unwrap
            }
        // 

        fn omega_max(&self) -> Omega {
            self._omega_max.unwrap_or(self.omega_start_stop)
        }

        fn set_omega_max(&mut self, omega : Omega) -> Result<(), DriveError> {
            if omega > self.omega_start_stop {
                Err(DriveError::OmegaMaxTooHigh)
            } else {
                self._omega_max = Some(omega); 
                Ok(())
            }
        }

        fn drive_mode(&self) -> &DriveMode {
            &self.mode
        }

        fn set_drive_mode<C : Controller>(&mut self, mode : DriveMode, ctrl : &mut C) -> Result<(), DriveError> {
            match mode {
                DriveMode::ConstOmega(omega, dir) => {
                    if omega > self.omega_max() {
                        return Err(DriveError::OmegaMaxTooHigh)
                    } 

                    ctrl.set_dir(dir);
                },
                DriveMode::ConstFactor(_, dir) => {
                    ctrl.set_dir(dir);
                },
                DriveMode::FixedDistance(delta, omega_out, _) => {
                    if omega_out > self.omega_max() {
                        return Err(DriveError::OmegaOutTooHigh)
                    }

                    self.distance = self.consts.steps_from_angle_abs(delta, self.microsteps);
                    self.distance_counter = 0;

                    ctrl.set_dir(delta.get_direction());
                },
                _ => { }
            };

            self.mode = mode;
            Ok(())
        }
    }
// 