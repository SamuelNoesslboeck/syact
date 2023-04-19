use std::sync::mpsc::{Sender, Receiver, channel}; 
use std::thread;

use serde::{Serialize, Deserialize};

use crate::ctrl::pin;
use crate::units::*;

/// A simple software PWM-signal 
#[derive(Debug)]
pub struct PWMOutput
{
    /// The pin of the PWM-signal
    pub pin : u8,

    t_ac : Time,
    t_in : Time,

    // Thread
    thr : Option<thread::JoinHandle<()>>,
    sender : Option<Sender<[Time; 2]>>,
    murderer: Option<Receiver<()>>,
}

impl PWMOutput 
{
    /// Create a new software PWM-signal at the given `pin`. Make sure the `pin` is not already in use
    pub fn new(pin : u8) -> Self {
        Self {
            pin: pin,

            t_ac: Time::NAN,
            t_in: Time::NAN,

            thr: None,
            sender: None,
            murderer: None
        }
    }

    /// Starts the thread for the signal
    pub fn start(&mut self) {
        let mut sys_pwm = pin::UniPin::new(self.pin).unwrap().into_output();

        let (sender, recv) : (Sender<[Time; 2]>, Receiver<[Time; 2]>) = channel();
        let (victim, murder) : (Sender<()>, Receiver<()>) = channel();

        let thr = thread::spawn(move || {
            let mut t_ac = Time::NAN;
            let mut t_in = Time::NAN;

            loop {
                if t_in.is_nan() {
                    let [ n_ac, n_in ] = recv.recv().unwrap();

                    if (n_ac.is_nan()) & (n_in.is_nan()) {
                        break;
                    }

                    t_ac = n_ac;
                    t_in = n_in;
                }

                match recv.try_recv() {
                    Ok([n_ac, n_in, ]) => {

                        if (n_ac.is_nan()) & (n_in.is_nan()) {
                            break;
                        }

                        t_ac = n_ac; 
                        t_in = n_in;
                    },
                    _ => { }
                }

                PWMOutput::pulse(&mut sys_pwm, t_ac, t_in);
            }

            victim.send(()).unwrap();
        }); 

        self.thr = Some(thr);
        self.sender = Some(sender);
        self.murderer = Some(murder);
    }

    /// Does a single pulse with the active time `t_ac` and inactive time `t_in`
    pub fn pulse(sys_pwm : &mut pin::UniOutPin, t_ac : Time, t_in : Time) {
        sys_pwm.set_high();
        thread::sleep(t_ac.into());
        sys_pwm.set_low(); 
        thread::sleep(t_in.into());
    }

    /// Get the current signal times (`t_ac`, `t_in`)
    #[inline]
    pub fn get_times(&self) -> [Time; 2] {
        [ self.t_ac, self.t_in ]
    }

    /// Sets the signal times (`t_ac`, `t_in`)
    #[inline]
    pub fn set_times(&mut self, t_ac : Time, t_in : Time) {
        self.t_ac = t_ac.max(Time::ZERO);
        self.t_in = t_in.max(Time::ZERO);
        
        if let Some(sender) = &mut self.sender {
            sender.send([ self.t_ac, self.t_in ]).unwrap();
        }
    }

    /// Get the signal times in period style (`t_ac`, `t_per`)
    #[inline]
    pub fn get_period(&self) -> [Time; 2] {
        [ self.t_ac, self.t_ac + self.t_in ]
    }

    /// Set the signal times in period style 
    /// - `t_ac` is the active time
    /// - `t_per` is the full period time
    /// 
    /// # Panics
    /// 
    /// The function panics if the given active time `t_ac` is bigger than the period time `t_per`
    #[inline]
    pub fn set_period(&mut self, t_ac : Time, t_per : Time) {
        if t_ac > t_per {
            panic!("Active time cannot be bigger than period time! (t_ac: {}, t_per: {})", t_ac, t_per);
        }

        self.set_times(
            t_ac,
            t_per - t_ac
        );
    }

    /// Get the signal times in frequency style (`freq`, `factor`)
    /// - `freq` is the freqency of the signal, meaning how many pulses there are per second
    /// - `factor` represents how much of the pulse is active time (values `0.0` to `1.0`)
    #[inline]
    pub fn get_freq(&self) -> (Omega, f32) {
        let [ t_ac, t_per ] = self.get_period();
        ( 1.0 / t_per, t_ac / t_per )
    }

    /// Set the signal times in frequency style
    /// - `freq` is the freqency of the signal, meaning how many pulses there are per second
    /// - `factor` represents how much of the pulse is active time (values `0.0` to `1.0`)
    /// 
    /// # Panics
    /// 
    /// The function panics if the given factor is out of range 
    #[inline]
    pub fn set_freq(&mut self, freq : Omega, factor : f32) {
        #[cfg(feature = "std")]
        if (1.0 < factor) & (0.0 > factor) {
            panic!("Bad factor value! {}", factor);
        }

        self.set_period(
            1.0 / freq * factor,
            1.0 / freq
        )
    }

    /// Stops the PWM-Signal and deletes the thread, can be started again if desired with `start()`
    pub fn stop(&mut self) {
        if let Some(sender) = &mut self.sender {
            sender.send([ Time::NAN, Time::NAN ]).unwrap();
        }

        self.thr = None;
        self.sender = None;

        if let Some(murder) = &mut self.murderer {
            murder.recv().unwrap();
        }

        self.murderer = None;
    }
}

impl Serialize for PWMOutput {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
        where
            S: serde::Serializer {
        serializer.serialize_u8(self.pin)
    }
}

impl<'de> Deserialize<'de> for PWMOutput {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
        where
            D: serde::Deserializer<'de> {
        Ok(PWMOutput::new(
            Deserialize::deserialize(deserializer)?
        ))
    }
}