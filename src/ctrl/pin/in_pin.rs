use embedded_hal::digital::v2::InputPin;
#[cfg(feature = "rasp")]
use rppal::gpio::InputPin;

/// Universal output pin structure for platform independency
#[derive(Debug)]
pub struct UniInPin {
    /// The pin control used
    #[cfg(feature = "rasp")]
    pub sys_pin : InputPin,

    /// The pin number, TODO: REMOVE, SHOULD BE HEADERLESS WRAPPER!
    pub pin : u8
}

// #####################################
// #    EMBEDDED-HAL IMPLEMENTATION    #
// #####################################
    // Implementation for no GPIO platforms (e.g. windows)
    //  -> Default value for pin value is `LOW`
    //  -> Implementation cannot fail
    #[cfg(not(any(feature = "rasp")))]
    impl InputPin for UniInPin {
        type Error = ();

        #[inline]
        fn is_low(&self) -> Result<bool, Self::Error> {
            Ok(true)
        }

        #[inline]
        fn is_high(&self) -> Result<bool, Self::Error> {
            Ok(false)
        }
    }

    // Implementation for Raspberry Pi devices
    //  -> Implementation cannot fail
    #[cfg(feature = "rasp")]
    impl InputPin for UniInPin {
        type Error = ();

        #[inline]
        fn is_low(&self) -> Result<bool, Self::Error> {
            Ok(self.sys_pin.is_low())
        }

        #[inline]
        fn is_high(&self) -> Result<bool, Self::Error> {
            Ok(self.sys_pin.is_high())
        }
    }
// 