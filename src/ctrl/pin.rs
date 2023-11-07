#[cfg(feature = "rasp")]
use rppal::gpio::{Gpio, Pin};

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
    /// The pin control used
    #[cfg(feature = "rasp")]
    pub sys_pin : Pin,

    /// The pin number
    pub pin : u8
}

impl UniPin {
    /// Create a new raspberry pi GPIO pin
    #[cfg(feature = "rasp")]
    pub fn new(pin : u8) -> Result<Self, crate::Error> {
        use crate::lib_error;

        let sys_pin = match Gpio::new() {
            Ok(gp) => match gp.get(pin) { 
                Ok(pin) => pin,
                Err(err) => return Err(lib_error(format!("{:?}", err)))
            },
            Err(err) => return Err(lib_error(format!("{:?}", err)))
        };
        
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
    #[cfg(feature = "rasp")]
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
    #[cfg(feature = "rasp")]
    pub fn into_output(self) -> UniOutPin {
        UniOutPin::new(
            self.sys_pin.into_output(),
            self.pin
        )
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