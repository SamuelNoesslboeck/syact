#[cfg(unix)]
use rppal::gpio::{Gpio, Pin, InputPin, OutputPin};

pub const ERR_PIN : u8 = 0xFF; 

#[derive(Debug)]
pub struct UniPin {
    #[cfg(unix)]
    sys_pin : Pin,

    pub pin : u8
}

#[derive(Debug)]
pub struct SimOutPin {
    #[cfg(unix)]
    sys_pin : OutputPin,
    #[cfg(windows)]
    state : bool, 

    pub pin : u8
}

#[derive(Debug)]
pub struct SimInPin {
    #[cfg(unix)]
    sys_pin : InputPin,

    pub pin : u8
}

impl UniPin {
    #[cfg(unix)]
    pub fn new(pin : u8) -> Result<Self, crate::Error> {
        let sys_pin = match Gpio::new() {
            Ok(gp) => match gp.get(pin) { 
                Ok(pin) => pin,
                Err(err) => return Err(std::io::Error::new(std::io::ErrorKind::Other, format!("{:?}", err)))
            },
            Err(err) => return Err(std::io::Error::new(std::io::ErrorKind::Other, format!("{:?}", err)))
        };

        Ok(Self {
            pin,
            sys_pin
        })
    }

    #[cfg(windows)]
    pub fn new(pin : u8) -> Result<Self, crate::Error> {
        Ok(Self {
            pin
        })
    }

    #[cfg(unix)]
    pub fn into_input(self) -> SimInPin {
        SimInPin {
            pin: self.pin,
            sys_pin: self.sys_pin.into_input()
        }
    }

    #[cfg(windows)]
    pub fn into_input(self) -> SimInPin {
        SimInPin {
            pin: self.pin
        }
    }

    #[cfg(unix)]
    pub fn into_output(self) -> SimOutPin {
        SimOutPin {
            pin: self.pin,
            sys_pin: self.sys_pin.into_output()
        }
    }

    #[cfg(windows)]
    pub fn into_output(self) -> SimOutPin {
        SimOutPin {
            pin: self.pin,
            state: false
        }
    }
}

impl SimInPin {
    #[cfg(unix)]
    #[inline]
    pub fn is_sim(&self) -> bool {
        false
    }

    #[cfg(windows)]
    #[inline]
    pub fn is_sim(&self) -> bool {
        true
    }

    #[cfg(windows)]
    #[inline]
    pub fn is_high(&self) -> bool {
        true
    }

    #[cfg(unix)]
    #[inline]
    pub fn is_high(&self) -> bool {
        self.sys_pin.is_high()
    }

    #[cfg(windows)]
    #[inline]
    pub fn is_low(&self) -> bool {
        false
    }

    #[cfg(unix)]
    #[inline]
    pub fn is_low(&self) -> bool {
        self.sys_pin.is_low()
    }
}

impl SimOutPin {
    #[cfg(unix)]
    #[inline]
    pub fn is_sim(&self) -> bool {
        false
    }

    #[cfg(windows)]
    #[inline]
    pub fn is_sim(&self) -> bool {
        true
    }
    
    #[cfg(windows)]
    #[inline]
    pub fn is_set_high(&self) -> bool {
        self.state
    }

    #[cfg(unix)]
    #[inline]
    pub fn is_set_high(&self) -> bool {
        self.sys_pin.is_set_high()
    }

    #[cfg(windows)]
    #[inline]
    pub fn is_set_low(&self) -> bool {
        !self.state
    }

    #[cfg(unix)]
    #[inline]
    pub fn is_set_low(&self) -> bool {
        self.sys_pin.is_set_low()
    }

    #[cfg(windows)]
    #[inline]
    pub fn set_high(&mut self) {
        self.state = true;
    }

    #[cfg(unix)]
    #[inline]
    pub fn set_high(&mut self) {
        self.sys_pin.set_high();
    }

    #[cfg(windows)]
    #[inline]
    pub fn set_low(&mut self) {
        self.state = false;
    }

    #[cfg(unix)]
    #[inline]
    pub fn set_low(&mut self) {
        self.sys_pin.set_low();
    }
}