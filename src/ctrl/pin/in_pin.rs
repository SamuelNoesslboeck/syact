use embedded_hal::digital::v2::InputPin;
#[cfg(feature = "rasp")]
use rppal::gpio::InputPin;

use crate::Setup;
use crate::ctrl::pin::ERR_PIN;

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
    cfg_if::cfg_if! { 
        if #[cfg(not(any(feature = "rasp")))] {
            impl UniInPin {
                pub fn new(pin : u8) -> Self {
                    Self { pin }
                }
            }

            impl Setup for UniInPin { }

            impl Default for UniInPin {
                fn default() -> Self {
                    Self {
                        pin: ERR_PIN
                    }
                }
            }

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
        } 
    }
    //

    // Implementation for Raspberry Pi devices
    //  -> Implementation cannot fail
    cfg_if::cfg_if! { 
        if #[cfg(feature = "rasp")] {
            impl UniInPin {
                pub fn new(pin : u8) -> Self {
                    Self {
                        pin,
                        sys_pin: None
                    }
                }
            }

            impl Default for UniInPin {
                fn default() -> Self {
                    Self {
                        pin: ERR_PIN,
                        sys_pin: None
                    }
                }
            }

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
        }
    }
    // 
// 