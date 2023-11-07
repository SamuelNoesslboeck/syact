use embedded_hal::digital::v2::InputPin;
#[cfg(feature = "rasp")]
use rppal::gpio::InputPin;

use crate::Setup;

/// Universal output pin structure for platform independency with `Setup`-Overhead
#[derive(Debug)]
pub struct UniInPin {
    /// The pin control used
    #[cfg(feature = "rasp")]
    pub sys_pin : Option<InputPin>,

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
        impl UniInPin {
            pub fn new(pin : u8) -> Self {
                Self { pin }
            }
        }

        #[cfg(not(any(feature = "rasp")))]
        impl Setup for UniInPin { }

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
    //

    // Implementation for Raspberry Pi devices
    //  -> Implementation cannot fail
        #[cfg(feature = "rasp")]
        impl UniInPin {
            pub fn new(pin : u8) -> Self {
                Self {
                    pin,
                    sys_pin: None
                }
            }
        }

        #[cfg(feature = "rasp")]
        impl Setup for UniInPin {
            fn setup(&mut self) -> Result<(), crate::Error> {
                self.sys_pin = match Gpio::new() {
                    Ok(gp) => match gp.get(self.pin) { 
                        Ok(pin) => pin,
                        Err(err) => return Err(lib_error(format!("{:?}", err)))
                    },
                    Err(err) => return Err(lib_error(format!("{:?}", err)))
                };

                Ok(())
            }
        }

        #[cfg(feature = "rasp")]
        impl InputPin for UniInPin {
            type Error = ();

            #[inline]
            fn is_low(&self) -> Result<bool, Self::Error> {
                if let Some(sys_pin) = &self.sys_pin {
                    Ok(self.sys_pin.is_low())
                } else {
                    panic!("Pin not setup yet! (Number: {})", self.pin)
                }
            }

            #[inline]
            fn is_high(&self) -> Result<bool, Self::Error> {
                if let Some(sys_pin) = &self.sys_pin {
                    Ok(self.sys_pin.is_high())
                } else {
                    panic!("Pin not setup yet! (Number: {})", self.pin)
                }
            }
        }
    // 
// 