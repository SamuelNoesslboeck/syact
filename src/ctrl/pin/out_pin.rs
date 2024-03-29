#[cfg(feature = "rasp")]
use rppal::gpio::OutputPin;

/// Universal ouput pin structure for platform independency
#[derive(Debug)]
pub struct UniOutPin {
    /// The pin control used
    #[cfg(feature = "rasp")]
    pub sys_pin : OutputPin,
    #[cfg(not(any(feature = "rasp")))]
    state : bool, 

    /// The pin number
    pub pin : u8
}

impl UniOutPin {
    /// Create 
    #[inline]
    #[cfg(feature = "rasp")]
    pub fn new(sys_pin : OutputPin, pin : u8) -> Self {
        Self {
            sys_pin, 
            pin
        }
    }

    /// Creates a new simulated output pin
    #[inline]
    #[cfg(not(any(feature = "rasp")))]
    pub fn new(state : bool, pin : u8) -> Self {
        Self {
            state, 
            pin
        }
    }

    /// Checks if the pin is simulated
    /// 
    /// Returns `false` for this configuration
    #[inline]
    #[cfg(feature = "rasp")]
    pub fn is_sim(&self) -> bool {
        false
    }

    /// Checks if the pin is simulated
    /// 
    /// Returns `true` for this configuration
    #[inline]
    #[cfg(not(any(feature = "rasp")))]
    pub fn is_sim(&self) -> bool {
        true
    }
    
    /// Checks if the pin is set to `HIGH`
    #[inline]
    #[cfg(not(any(feature = "rasp")))]
    pub fn is_set_high(&self) -> bool {
        self.state
    }

    /// Checks if the pin is set to `HIGH`
    #[inline]
    #[cfg(feature = "rasp")]
    pub fn is_set_high(&self) -> bool {
        self.sys_pin.is_set_high()
    }

    /// Checks if the pin is set to `LOW`
    #[inline]
    #[cfg(not(any(feature = "rasp")))]
    pub fn is_set_low(&self) -> bool {
        !self.state
    }

    /// Checks if the pin is set to `LOW`
    #[inline]
    #[cfg(feature = "rasp")]
    pub fn is_set_low(&self) -> bool {
        self.sys_pin.is_set_low()
    }

    /// Set the pin to `HIGH`
    #[inline]
    #[cfg(not(any(feature = "rasp")))]
    pub fn set_high(&mut self) {
        self.state = true;
    }

    /// Set the pin to `HIGH`
    #[inline]
    #[cfg(feature = "rasp")]
    pub fn set_high(&mut self) {
        self.sys_pin.set_high();
    }

    /// Set the pin to `lOW`
    #[inline]
    #[cfg(not(any(feature = "rasp")))]
    pub fn set_low(&mut self) {
        self.state = false;
    }

    /// Set the pin to `lOW`
    #[inline]
    #[cfg(feature = "rasp")]
    pub fn set_low(&mut self) {
        self.sys_pin.set_low();
    }

    /// Sets the pin to a boolean value 
    /// - `true` for high
    /// - `false` for low
    pub fn set(&mut self, val : bool) {
        if val {
            self.set_high()
        } else {
            self.set_low()
        }
    }
}
