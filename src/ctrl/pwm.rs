use std::sync::mpsc::{Sender, Receiver, channel}; 
use std::thread;

use gpio::GpioOut;
use serde::{Serialize, Deserialize};

use crate::ctrl::types::RaspPin;
use crate::units::*;

#[derive(Debug)]
pub struct PWMOutput
{
    pub pin : u16,

    t_ac : Time,
    t_in : Time,

    // Thread
    pub thr : thread::JoinHandle<()>,
    pub sender : Sender<[Time; 2]>
}

impl PWMOutput 
{
    pub fn spawn(pin : u16) -> Self {
        let mut sys_pwm = match gpio::sysfs::SysFsGpioOutput::open(pin.clone()) {
            Ok(val) => RaspPin::Output(val),
            Err(_) => {
                // println!("Failed to open pin! {} ", pin);    // TODO: Remove dirty stuff
                RaspPin::ErrPin
            }
        };

        let (sender, recv) : (Sender<[Time; 2]>, Receiver<[Time; 2]>) = channel();

        let thr = thread::spawn(move || {
            let mut t_ac = Time::NAN;
            let mut t_in = Time::NAN;

            loop {
                if t_in.is_nan() {
                    let [ n_ac, n_in ] = recv.recv().unwrap();
                    // println!("Recv msg {:?} {:?}", n_ac, n_in);
                    t_ac = n_ac;
                    t_in = n_in;
                }

                match recv.try_recv() {
                    Ok([n_ac, n_in, ]) => {
                        // println!("Recv msg {:?} {:?}", n_ac, n_in); 
                        t_ac = n_ac; 
                        t_in = n_in;
                    },
                    _ => { }
                }

                PWMOutput::pulse(&mut sys_pwm, t_ac, t_in);
            }
        }); 

        PWMOutput {
            thr,

            t_ac: Time::NAN,
            t_in: Time::NAN,

            sender,
            pin
        }
    }

    pub fn pulse(sys_pwm : &mut RaspPin, t_ac : Time, t_in : Time) {
        match sys_pwm {
            RaspPin::Output(pin) => {
                pin.set_high().unwrap();
                thread::sleep(t_ac.into());
                pin.set_low().unwrap(); 
                thread::sleep(t_in.into());
            },
            _ => { }
        }
    }

    #[inline]
    pub fn get_times(&self) -> [Time; 2] {
        [ self.t_ac, self.t_in ]
    }

    #[inline]
    pub fn set_times(&mut self, t_ac : Time, t_in : Time) {
        self.t_ac = t_ac.max(Time::ZERO);
        self.t_in = t_in.max(Time::ZERO);
        
        self.sender.send([ self.t_ac, self.t_in ]).unwrap();
    }

    #[inline]
    pub fn get_period(&self) -> [Time; 2] {
        [ self.t_ac, self.t_ac + self.t_in ]
    }

    #[inline]
    pub fn set_period(&mut self, t_ac : Time, t_per : Time) {
        self.set_times(
            t_ac,
            t_per - t_ac
        );
    }

    #[inline]
    pub fn get_freq(&self) -> (Omega, f32) {
        let [ t_ac, t_per ] = self.get_period();
        ( 1.0 / t_per, t_ac / t_per )
    }

    #[inline]
    pub fn set_freq(&mut self, freq : Omega, perc : f32) {
        self.set_period(
            1.0 / freq * perc,
            1.0 / freq
        )
    }
}

impl Serialize for PWMOutput {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
        where
            S: serde::Serializer {
        serializer.serialize_u16(self.pin)
    }
}

impl<'de> Deserialize<'de> for PWMOutput {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
        where
            D: serde::Deserializer<'de> {
        Ok(PWMOutput::spawn(
            Deserialize::deserialize(deserializer)?
        ))
    }
}