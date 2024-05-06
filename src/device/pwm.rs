use core::fmt::Debug;
use core::marker::PhantomData;
use std::sync::mpsc::{Sender, Receiver, channel}; 
use std::thread::{self, JoinHandle};

use embedded_hal::digital::OutputPin;
// use serde::{Serialize, Deserialize};
use syunit::*;

use crate::{Setup, Dismantle};

/// A simple software-made PWM-signal 
#[derive(Debug)]
pub struct SoftwarePWM<P : OutputPin + Send> {
    /// Active time in a cycle (high)
    t_ac : Time,
    /// Inactive time in a cycle (low)
    t_in : Time,

    // Thread
    _thr : JoinHandle<()>,
    sender : Sender<[Time; 2]>,
    murderer: Receiver<()>,

    // Helper data
    __pd : PhantomData<P>
}

impl<P : OutputPin + Send + 'static> SoftwarePWM<P> {
    /// Create a new software PWM-signal at the given `pin`. Make sure the `pin` is not already in use!
    /// 
    /// # Setup
    /// 
    /// Once the setup function is called, the thread and the pin will be crated
    pub fn new(mut pin : P) -> Self {
        // The thread communication structures
        let (sender, recv) : (Sender<[Time; 2]>, Receiver<[Time; 2]>) = channel();
        let (victim, murderer) : (Sender<()>, Receiver<()>) = channel();

        // The thread
        let thr = thread::spawn(move || {
            // Initialize the values to errors
            let mut t_ac = Time::NAN;
            let mut t_in = Time::NAN;

            loop {
                // Force the thread to wait for new values if they are error `NAN` values
                if t_in.is_nan() {
                    let [ n_ac, n_in ] = recv.recv().unwrap();

                    if (n_ac.is_nan()) & (n_in.is_nan()) {
                        break;
                    }

                    t_ac = n_ac;
                    t_in = n_in;
                }

                // Checks weither a new message is available
                if let Ok([n_ac, n_in]) = recv.try_recv() {
                    if (n_ac.is_nan()) & (n_in.is_nan()) {
                        break;
                    }

                    t_ac = n_ac; 
                    t_in = n_in;
                }

                // Crates pulse
                SoftwarePWM::pulse(&mut pin, t_ac, t_in).unwrap();  // TODO: Unwrap
            }

            victim.send(()).unwrap();
        }); 

        Self {
            t_ac: Time::NAN,
            t_in: Time::NAN,

            _thr: thr,
            sender,
            murderer,
            
            __pd: Default::default()
        }
    }

    // Owned functions
        pub fn with_freq(mut self, freq : Velocity) -> Self {
            self.set_freq(freq, 0.0);
            self
        }

        pub fn with_period_time(mut self, time : Time) -> Self {
            self.set_times(Time::ZERO, time);
            self
        }
    // 

    // /// Starts the thread for the signal and creates the pin for the output signal
    // pub fn start(&mut self, sys_pwm : P) -> Result<(), crate::Error> {


    //     Ok(())
    // }

    /// Does a single pulse with the active time `t_ac` and inactive time `t_in`
    pub fn pulse(pin : &mut P, t_ac : Time, t_in : Time) -> Result<(), P::Error> {
        pin.set_high()?;
        thread::sleep(t_ac.into());
        pin.set_low()?;
        thread::sleep(t_in.into());
        Ok(())
    }

    // Times
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
            
            self.sender.send([ self.t_ac, self.t_in ]).unwrap();
        }
    //
    
    // Period
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
    //

    // Freq
        /// Get the signal times in frequency style (`freq`, `factor`)
        /// - `freq` is the freqency of the signal, meaning how many pulses there are per second
        /// - `factor` represents how much of the pulse is active time (values `0.0` to `1.0`)
        #[inline]
        pub fn get_freq(&self) -> (Velocity, f32) {
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
        pub fn set_freq(&mut self, freq : Velocity, factor : f32) {
            if (1.0 < factor) & (0.0 > factor) {
                panic!("Bad factor value! {}", factor);
            }

            self.set_period(
                1.0 / freq * factor,
                1.0 / freq
            )
        }
    //

    /// Stops the PWM-Signal and deletes the thread, can be started again if desired with `start()`
    pub fn stop(&mut self) -> Result<(), crate::Error> {
        self.sender.send([ Time::NAN, Time::NAN ])?;
        self.murderer.recv()?;

        Ok(())
    }
}

// Setup and dismantle
impl<P : OutputPin + Send> Setup for SoftwarePWM<P> {
    fn setup(&mut self) -> Result<(), crate::Error> {
        Ok(())
    }
}

impl<P : OutputPin + Send + 'static> Dismantle for SoftwarePWM<P> {
    fn dismantle(&mut self) -> Result<(), crate::Error> {
        self.stop()
    }
}

// Embedded-HAL implementations
    // #[derive(Clone, Debug)]
    // pub enum SoftwarePWMError<P : OutputPin> {
    //     Pin(P::Error)
    // }

    impl<P : OutputPin + Send> embedded_hal::pwm::ErrorType for SoftwarePWM<P> {
        type Error = embedded_hal::pwm::ErrorKind;
    }

    impl<P : OutputPin + Send + 'static> embedded_hal::pwm::SetDutyCycle for SoftwarePWM<P> {
        fn max_duty_cycle(&self) -> u16 {
            u16::MAX
        }

        fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
            let [_, period] = self.get_period();
            self.set_period(((duty as f32) / (self.max_duty_cycle() as f32)) * period, period);
            Ok(())
        }
    }
// 

// JSON Input/Output
    // impl<P : OutputPin> Serialize for SoftwarePWM<P> {
    //     fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    //         where
    //             S: serde::Serializer {
    //         serializer.serialize_u8(self._pin)
    //     }
    // }

    // impl<'de, P : OutputPin> Deserialize<'de> for SoftwarePWM<P> {
    //     fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    //         where
    //             D: serde::Deserializer<'de> {
    //         Ok(SoftwarePWM::new(
    //             Deserialize::deserialize(deserializer)?
    //         ))
    //     }
    // }
// 