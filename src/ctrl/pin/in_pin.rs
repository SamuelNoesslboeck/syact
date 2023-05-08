#[cfg(feature = "rasp")]
use rppal::gpio::InputPin;

/// Universal output pin structure for platform independency
#[derive(Debug)]
pub struct UniInPin {
    #[cfg(feature = "rasp")]
    sys_pin : InputPin,

    /// The pin number
    pub pin : u8
}

impl UniInPin {
    // Status
        /// Checks if the pin is simulated
        /// 
        /// Returns `false` for this configuration
        #[inline(always)]
        #[cfg(feature = "rasp")]
        pub fn is_sim(&self) -> bool {
            false
        }

        /// Checks if the pin is simulated
        /// 
        /// Returns `true` for this configuration
        #[inline(always)]
        #[cfg(not(any(feature = "rasp")))]
        pub fn is_sim(&self) -> bool {
            true
        }
    // 

    /// Checks if the pin receives a `HIGH` signal
    #[inline]
    #[cfg(not(any(feature = "rasp")))]
    pub fn is_high(&self) -> bool {
        true
    }

    /// Checks if the pin receives a `HIGH` signal
    #[inline]
    #[cfg(feature = "rasp")]
    pub fn is_high(&self) -> bool {
        self.sys_pin.is_high()
    }

    /// Checks if the pin receives a `LOW` signal
    #[inline]
    #[cfg(not(any(feature = "rasp")))]
    pub fn is_low(&self) -> bool {
        false
    }

    /// Checks if the pin receives a `LOW` signal
    #[inline]
    #[cfg(feature = "rasp")]
    pub fn is_low(&self) -> bool {
        self.sys_pin.is_low()
    }
}