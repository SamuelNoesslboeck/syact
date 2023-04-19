#[cfg(featue = "rasp")]
use rppal::gpio::{Gpio, Pin, InputPin, OutputPin};

/// Error pin value
pub const ERR_PIN : u8 = 0xFF; 

/// Universal pin struct for platform independency
#[derive(Debug)]
pub struct UniPin {
    #[cfg(featue = "rasp")]
    sys_pin : Pin,

    /// The pin number
    pub pin : u8
}

#[derive(Debug)]
pub struct UniOutPin {
    #[cfg(featue = "rasp")]
    sys_pin : OutputPin,
    #[cfg(windows)]
    state : bool, 

    /// The pin number
    pub pin : u8
}

#[derive(Debug)]
pub struct UniInPin {
    #[cfg(featue = "rasp")]
    sys_pin : InputPin,

    /// The pin number
    pub pin : u8
}

impl UniPin {
    #[cfg(featue = "rasp")]
    pub fn new(pin : u8) -> Result<Self, crate::Error> {
        let sys_pin = match Gpio::new() {
            Ok(gp) => match gp.get(pin) { 
                Ok(pin) => pin,
                Err(err) => return Err(std::io::Error::new(std::io::ErrorKind::Other, format!("{:?}", err)))
            },
            Err(err) => return Err(std::io::Error::new(std::io::ErrorKind::Other, format!("{:?}", err)))
        };
        
        println!(" -> Pin successfully setup! [Pin: {}]", pin);
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

    #[cfg(featue = "rasp")]
    pub fn into_input(self) -> UniInPin {
        UniInPin {
            pin: self.pin,
            sys_pin: self.sys_pin.into_input()
        }
    }

    #[cfg(windows)]
    pub fn into_input(self) -> UniInPin {
        UniInPin {
            pin: self.pin
        }
    }

    #[cfg(featue = "rasp")]
    pub fn into_output(self) -> UniOutPin {
        UniOutPin {
            pin: self.pin,
            sys_pin: self.sys_pin.into_output()
        }
    }

    #[cfg(windows)]
    pub fn into_output(self) -> UniOutPin {
        UniOutPin {
            pin: self.pin,
            state: false
        }
    }
}

impl UniInPin {
    // Status
        #[cfg(featue = "rasp")]
        #[inline]
        pub fn is_sim(&self) -> bool {
            false
        }

        #[cfg(windows)]
        #[inline]
        pub fn is_sim(&self) -> bool {
            true
        }
    // 

    #[cfg(windows)]
    #[inline]
    pub fn is_high(&self) -> bool {
        true
    }

    #[cfg(featue = "rasp")]
    #[inline]
    pub fn is_high(&self) -> bool {
        self.sys_pin.is_high()
    }

    #[cfg(windows)]
    #[inline]
    pub fn is_low(&self) -> bool {
        false
    }

    #[cfg(featue = "rasp")]
    #[inline]
    pub fn is_low(&self) -> bool {
        self.sys_pin.is_low()
    }
}

impl UniOutPin {
    #[cfg(featue = "rasp")]
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

    #[cfg(featue = "rasp")]
    #[inline]
    pub fn is_set_high(&self) -> bool {
        self.sys_pin.is_set_high()
    }

    #[cfg(windows)]
    #[inline]
    pub fn is_set_low(&self) -> bool {
        !self.state
    }

    #[cfg(featue = "rasp")]
    #[inline]
    pub fn is_set_low(&self) -> bool {
        self.sys_pin.is_set_low()
    }

    #[cfg(windows)]
    #[inline]
    pub fn set_high(&mut self) {
        self.state = true;
    }

    #[cfg(featue = "rasp")]
    #[inline]
    pub fn set_high(&mut self) {
        self.sys_pin.set_high();
    }

    #[cfg(windows)]
    #[inline]
    pub fn set_low(&mut self) {
        self.state = false;
    }

    #[cfg(featue = "rasp")]
    #[inline]
    pub fn set_low(&mut self) {
        self.sys_pin.set_low();
    }
}
