use std::sync::mpsc::{Sender, Receiver, channel}; 
use std::thread;
use std::time::Duration;

use gpio::GpioOut;

use crate::ctrl::types::RaspPin;

pub struct PWMSignal
{
    pub pin : u16,

    t_ac : f32,
    t_in : f32,

    // Thread
    pub thr : thread::JoinHandle<()>,
    pub sender : Sender<[f32; 2]>
}

impl PWMSignal 
{
    pub fn spawn(pin : u16) -> Self {
        let mut sys_pwm = match gpio::sysfs::SysFsGpioOutput::open(pin.clone()) {
            Ok(val) => RaspPin::Output(val),
            Err(_) => RaspPin::ErrPin
        };

        let (sender, recv) : (Sender<[f32; 2]>, Receiver<[f32; 2]>) = channel();

        let thr = thread::spawn(move || {
            let mut t_ac = f32::NAN;
            let mut t_in = f32::NAN;

            loop {
                if t_in.is_nan() {
                    let [ n_ac, n_in ] = recv.recv().unwrap();
                    t_ac = n_ac;
                    t_in = n_in;
                }

                match recv.try_recv() {
                    Ok([n_ac, n_in, ]) => { 
                        t_ac = n_ac; 
                        t_in = n_in;
                    },
                    _ => { }
                }

                PWMSignal::pulse(&mut sys_pwm, t_ac, t_in);
            }
        }); 

        PWMSignal {
            thr,

            t_ac: f32::NAN,
            t_in: f32::NAN,

            sender,
            pin
        }
    }

    pub fn pulse(sys_pwm : &mut RaspPin, t_ac : f32, t_in : f32) {
        match sys_pwm {
            RaspPin::Output(pin) => {
                pin.set_high().unwrap();
                thread::sleep(Duration::from_secs_f32(t_ac));
                pin.set_low().unwrap(); 
                thread::sleep(Duration::from_secs_f32(t_in));
            },
            _ => { }
        }
    }

    #[inline]
    pub fn get_times(&self) -> [f32; 2] {
        [ self.t_ac, self.t_in ]
    }

    #[inline]
    pub fn set_times(&mut self, t_ac : f32, t_in : f32) {
        self.t_ac = t_ac.max(0.0);
        self.t_in = t_in.max(0.0);
        
        self.sender.send([ self.t_ac, self.t_in ]).unwrap();
    }

    #[inline]
    pub fn get_period(&self) -> [f32; 2] {
        [ self.t_ac, self.t_ac + self.t_in ]
    }

    #[inline]
    pub fn set_period(&mut self, t_ac : f32, t_per : f32) {
        self.set_times(
            t_ac,
            t_per - t_ac
        );
    }

    #[inline]
    pub fn get_freq(&self) -> [f32; 2] {
        let [ t_ac, t_per ] = self.get_period();
        [ 1.0 / t_per, t_ac / t_per ]
    }

    #[inline]
    pub fn set_freq(&mut self, freq : f32, perc : f32) {
        self.set_period(
            1.0 / freq * perc,
            1.0 / freq
        )
    }
}
