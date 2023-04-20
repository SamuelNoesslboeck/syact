#[cfg(featue = "rasp")]
use rppal::gpio::{Gpio, Pin, InputPin, OutputPin};

// Submodlues
mod in_pin;
pub use in_pin::UniInPin;

mod out_pin;
pub use out_pin::UniOutPin;

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

impl UniPin {
    /// Create a new raspberry pi GPIO pin
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

    /// Create a new simulated IO pin
    #[inline]
    #[cfg(not(any(feature = "rasp")))]
    pub fn new(pin : u8) -> Result<Self, crate::Error> {
        Ok(Self {
            pin
        })
    }

    /// Convert the pin into an input pin
    #[inline]
    #[cfg(featue = "rasp")]
    pub fn into_input(self) -> UniInPin {
        UniInPin {
            pin: self.pin,
            sys_pin: self.sys_pin.into_input()
        }
    }

    /// Convert the pin into an input pin
    #[inline]
    #[cfg(not(any(feature = "rasp")))]
    pub fn into_input(self) -> UniInPin {
        UniInPin {
            pin: self.pin
        }
    }

    /// Convert the pin into an output pin
    #[inline]
    #[cfg(featue = "rasp")]
    pub fn into_output(self) -> UniOutPin {
        UniOutPin::new(
            self.sys_pin.into_output(),
            self.pin
        );
    }

    /// Convert the pin into an output pin
    #[cfg(not(any(feature = "rasp")))]
    pub fn into_output(self) -> UniOutPin {
        UniOutPin::new(
            false,
            self.pin
        )
    }
}