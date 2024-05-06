use syunit::*;

use crate::math::movements::DefinedActuator;
use crate::{StepperConst, ActuatorVars, StepperConfig};
use crate::act::stepper::{StepError, Controller};
use crate::data::MicroSteps;
use crate::math;

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub enum DriveMode {
    ConstOmega(Velocity, Direction),
    ConstFactor(Factor, Direction),
    FixedDistance(Delta, Velocity, Factor),
    Stop,
    Inactive
}

#[derive(Clone, Debug)]
pub enum DriveError {
    OmegaMaxTooHigh,
    OmegaOutTooHigh,
    Overload,
    LimitReached,
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

        fn dir(&self) -> Direction;

        fn set_overload_curret(&mut self, current : Option<f32>) -> Result<(), DriveError>;
    //

    // Loads
        fn apply_gen_force(&mut self, force : Force) -> Result<(), DriveError>;

        /// Value positive in CW direction
        fn apply_dir_force(&mut self, force : Force) -> Result<(), DriveError>;

        fn apply_inertia(&mut self, inertia : Inertia);
    //

    // Velocity max
        fn omega_max(&self) -> Velocity;

        fn set_omega_max(&mut self, omega : Velocity) -> Result<(), DriveError>;
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
        _vars : ActuatorVars,
        _config : StepperConfig,

        // Speeds
        omega_start_stop : Velocity,
        _omega_max : Option<Velocity>,

        // Cache
        _microsteps : MicroSteps,
        mode : DriveMode,
        _step_angle : Delta, 
        _dir : Direction,

        distance : u64,
        distance_counter : u64
    }

    impl StartStopBuilder {
        pub fn update_start_stop(&mut self) -> Result<(), DriveError> {
            self.omega_start_stop = math::kin::omega_start_stop(
                self._vars.force_after_load_lower(
                    self.consts.torque_overload(self._config.overload_current)
                ).ok_or(DriveError::Overload)?, 
                self._vars.inertia_after_load(self.consts.inertia_motor), 
                self.consts.number_steps * self._microsteps
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
            }.map(|omega| self.consts.step_time(omega, self._microsteps))
        }
    }

    impl StepperBuilder for StartStopBuilder {
        // General constructors
            fn new(consts : StepperConst) -> Self
            where 
                Self: Sized 
            {
                let mut _self = Self {
                    _vars: ActuatorVars::ZERO,
                    _config: StepperConfig::GEN,

                    omega_start_stop: Velocity::INFINITY,
                    _omega_max: None,
                    _microsteps: MicroSteps::default(),

                    distance: 0,
                    distance_counter: 0,
                    _step_angle: consts.step_angle(MicroSteps::default()),
                    _dir: Direction::default(),

                    mode: DriveMode::Inactive,

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
                &self._vars
            }

            fn config(&self) -> &StepperConfig {
                &self._config
            }
            
            fn set_config(&mut self, config : StepperConfig) {
                self._config = config;
            }

            fn microsteps(&self) -> MicroSteps {
                self._microsteps
            }

            fn set_microsteps(&mut self, microsteps : MicroSteps) {
                self._step_angle = self.consts.step_angle(microsteps);
                self._microsteps = microsteps;
            }

            fn step_angle(&self) -> Delta {
                self._step_angle
            }

            fn dir(&self) -> Direction {
                self._dir
            }

            fn set_overload_curret(&mut self, current : Option<f32>) -> Result<(), DriveError> {
                self._config.overload_current = current;
                self.update_start_stop()
            }
        // 

        // Loads
            fn apply_gen_force(&mut self, force : Force) -> Result<(), DriveError> {
                self._vars.force_load_gen = force;
                self.update_start_stop()
            }

            fn apply_dir_force(&mut self, force : Force) -> Result<(), DriveError> {
                self._vars.force_load_dir = force;
                self.update_start_stop()
            }
            
            fn apply_inertia(&mut self, inertia : Inertia) {
                self._vars.inertia_load = inertia;
                self.update_start_stop().unwrap();      // Save unwrap
            }
        // 

        fn omega_max(&self) -> Velocity {
            self._omega_max.unwrap_or(self.omega_start_stop)
        }

        fn set_omega_max(&mut self, omega : Velocity) -> Result<(), DriveError> {
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

                    self._dir = dir;
                    ctrl.set_dir(dir);
                },
                DriveMode::ConstFactor(_, dir) => {
                    self._dir = dir;
                    ctrl.set_dir(dir);
                },
                DriveMode::FixedDistance(delta, omega_out, _) => {
                    if omega_out > self.omega_max() {
                        return Err(DriveError::OmegaOutTooHigh)
                    }

                    self.distance = self.consts.steps_from_angle_abs(delta, self._microsteps);
                    self.distance_counter = 0;

                    if delta >= Delta::ZERO {
                        self._dir = Direction::CW;
                    } else {
                        self._dir = Direction::CCW;
                    }
                    ctrl.set_dir(self._dir);
                },
                _ => { }
            };

            self.mode = mode;
            Ok(())
        }
    }

    impl DefinedActuator for StartStopBuilder {
        fn ptp_time_for_distance(&self, gamma_0 : Gamma, gamma_t : Gamma) -> Time {
            (gamma_t / gamma_0) / self.omega_max()
        }
    }

    /*
    pub struct ComplexStartStopBuilder {
        _consts : StepperConst,
        _vars : ActuatorVars,
        _config : StepperConfig,

        // Speeds
        omega_abs_max : Velocity,
        _omega_max : Option<Velocity>,

        // Cache
        microsteps : MicroSteps,
        mode : DriveMode,
        _step_angle : Delta, 
        _dir : Direction,

        // Speed
        speed_levels : Vec<Velocity>,
        time_sums : Vec<Time>,

        distance : u64,
        distance_counter : u64
    }

    impl ComplexStartStopBuilder {
        pub fn update(&mut self) -> Result<(), DriveError> {
            self.omega_abs_max = math::kin::start_stop(
                self._vars.force_after_load_lower(
                    self._consts.torque_overload(self._config.overload_current)
                ).ok_or(DriveError::Overload)?,  
                self._vars.inertia_after_load(self._consts.inertia_motor), 
                self._consts.number_steps
            );

            // Reset omega max if it is too high
            if let Some(omega_max) = self._omega_max {
                if omega_max >= self.omega_abs_max {
                    self._omega_max = None;
                }
            }

            Ok(())
        }
    }

    impl Iterator for ComplexStartStopBuilder {
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
            }.map(|omega| self._consts.step_time(omega, self.microsteps))
        }
    }

    impl StepperBuilder for ComplexStartStopBuilder {
        // General constructors
            fn new(consts : StepperConst) -> Self
            where 
                Self: Sized 
            {
                let mut _self = Self {
                    _vars: ActuatorVars::ZERO,
                    _config: StepperConfig::GEN,

                    omega_abs_max: Velocity::INFINITY,
                    _omega_max: None,
                    microsteps: MicroSteps::default(),

                    distance: 0,
                    distance_counter: 0,
                    _step_angle: consts.step_angle(MicroSteps::default()),
                    _dir: Direction::default(),

                    speed_levels: Vec::new(),
                    time_sums: Vec::new(),

                    mode: DriveMode::Inactive,

                    _consts: consts
                };

                // TODO: Enable error
                _self.update().unwrap();

                _self
            }
        // 

        // Data
            fn consts(&self) -> &StepperConst {
                &self._consts
            }

            fn vars(&self) -> &ActuatorVars {
                &self._vars
            }

            fn config(&self) -> &StepperConfig {
                &self._config
            }
            
            fn set_config(&mut self, config : StepperConfig) {
                self._config = config;
            }

            fn microsteps(&self) -> MicroSteps {
                self.microsteps
            }

            fn set_microsteps(&mut self, microsteps : MicroSteps) {
                self._step_angle = self._consts.step_angle(microsteps);
                self.microsteps = microsteps;
            }

            fn step_angle(&self) -> Delta {
                self._step_angle
            }

            fn dir(&self) -> Direction {
                self._dir
            }

            fn set_overload_curret(&mut self, current : Option<f32>) -> Result<(), DriveError> {
                self._config.overload_current = current;
                self.update()
            }
        // 

        // Loads
            fn apply_gen_force(&mut self, force : Force) -> Result<(), DriveError> {
                self._vars.force_load_gen = force;
                self.update()
            }

            fn apply_dir_force(&mut self, force : Force) -> Result<(), DriveError> {
                self._vars.force_load_dir = force;
                self.update()
            }
            
            fn apply_inertia(&mut self, inertia : Inertia) {
                self._vars.inertia_load = inertia;
                self.update().unwrap();      // Save unwrap
            }
        // 

        fn omega_max(&self) -> Velocity {
            self._omega_max.unwrap_or(self.omega_abs_max)
        }

        fn set_omega_max(&mut self, omega : Velocity) -> Result<(), DriveError> {
            if omega > self.omega_abs_max {
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

                    self._dir = dir;
                    ctrl.set_dir(dir);
                },
                DriveMode::ConstFactor(_, dir) => {
                    self._dir = dir;
                    ctrl.set_dir(dir);
                },
                DriveMode::FixedDistance(delta, omega_out, _) => {
                    if omega_out > self.omega_max() {
                        return Err(DriveError::OmegaOutTooHigh)
                    }

                    self.distance = self._consts.steps_from_angle_abs(delta, self.microsteps);
                    self.distance_counter = 0;

                    self._dir = delta.get_direction();
                    ctrl.set_dir(self._dir);
                },
                _ => { }
            };

            self.mode = mode;
            Ok(())
        }
    }

    impl DefinedActuator for ComplexStartStopBuilder {
        fn ptp_time_for_distance(&self, gamma_0 : Gamma, gamma_t : Gamma) -> Time {
            (gamma_t / gamma_0) / self.omega_max()
        }
    }
    */
// 