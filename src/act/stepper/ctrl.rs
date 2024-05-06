use core::time::Duration;
use std::time::Instant;

use embedded_hal::digital::OutputPin;
use syunit::*;

use crate::{Setup, Dismantle};
use crate::act::stepper::StepError;

// Constants
pub const STEP_PULSE_TIME : Time = Time(1.0 / 40000.0);
const STEP_PULSE_DUR : Duration = Duration::from_micros(25);

/// A controller for the logics of a stepper motor
pub trait Controller {
    fn step_with(&mut self, t_len : Time, t_pause : Time);

    fn step(&mut self, time : Time) {
        if time < (STEP_PULSE_TIME * 2.0) {
            // TODO: Notify about fallback
            self.step_with(STEP_PULSE_TIME, STEP_PULSE_TIME);
        } else {
            self.step_with(STEP_PULSE_TIME, time - STEP_PULSE_TIME)
        }
    }

    fn step_no_wait(&mut self, t_total : Time) -> Result<(), StepError>;

    fn step_final(&mut self) -> Result<(), StepError>{
        self.step_no_wait(2.0 * STEP_PULSE_TIME)
    }

    fn dir(&self) -> Direction;

    fn set_dir(&mut self, dir : Direction);
}

#[derive(Debug)]
pub struct GenericPWM<S : OutputPin, D : OutputPin> {
    dir : Direction,

    pin_step : S,
    pin_dir : D,

    t_pause : Duration,
    pause_stamp : Instant
}

impl<S : OutputPin, D : OutputPin> GenericPWM<S, D> {
    pub fn new(pin_step : S, pin_dir : D) -> Result<Self, crate::Error> {
        Ok(Self {
            dir: Direction::CW,
            
            pin_step,
            pin_dir,

            t_pause: Duration::ZERO,
            pause_stamp: Instant::now()
        }) 
    }

    // pub fn new_gen() -> Self {
    //     Self::new(ERR_PIN, ERR_PIN).unwrap()
    // }

    // pub fn pin_step(&self) -> u8 {
    //     self.pin_step.pin
    // }

    // pub fn pin_dir(&self) -> u8 {
    //     self.pin_dir.pin
    // }
}

impl<S : OutputPin, D : OutputPin> Setup for GenericPWM<S, D> {
    fn setup(&mut self) -> Result<(), crate::Error> {
        // self.pin_step.setup()?; 
        // self.pin_dir.setup()?;
        // TODO: Add pins setup

        Ok(())
    }
}

impl<S : OutputPin, D : OutputPin> Dismantle for GenericPWM<S, D> {
    fn dismantle(&mut self) -> Result<(), crate::Error> {
        // TODO: Add pins dismantle
        Ok(())
    }
}

impl<S : OutputPin, D : OutputPin> Controller for GenericPWM<S, D> {
    fn step_with(&mut self, t_len : Time, t_pause : Time) {
        self.pin_step.set_high().unwrap();
        spin_sleep::sleep(t_len.into());
        self.pin_step.set_low().unwrap();
        spin_sleep::sleep(t_pause.into());
    }

    fn step_no_wait(&mut self, t_total : Time) -> Result<(), StepError> {
        // TODO: Maybe create fallbacks or move to Result?
        if t_total <= STEP_PULSE_TIME {
            return Err(StepError::TimeTooShort(t_total));
        }

        if !t_total.is_normal() {
            return Err(StepError::TimeIsIncorrect(t_total));
        }

        let elapsed = self.pause_stamp.elapsed();
        if elapsed < self.t_pause {
                spin_sleep::sleep(self.t_pause - elapsed);      // Makes t_pause = elapsed
            }

        if self.t_pause < STEP_PULSE_DUR {
                spin_sleep::sleep(STEP_PULSE_DUR - self.t_pause);
            }

        self.pin_step.set_high().unwrap();
        spin_sleep::sleep(STEP_PULSE_DUR);
        self.pin_step.set_low().unwrap();

        self.pause_stamp = Instant::now();
        self.t_pause = (t_total - STEP_PULSE_TIME).into();

        Ok(())
    }

    fn dir(&self) -> Direction {
        self.dir
    }

    fn set_dir(&mut self, dir : Direction) {
        self.dir = dir;
        self.pin_dir.set_state(self.dir.as_bool().into()).unwrap();
    }
}