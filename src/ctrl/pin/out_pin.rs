#[cfg(feature = "rasp")]
use rppal::gpio::{Gpio, OutputPin};

use embedded_hal::digital::v2::PinState;

use crate::Setup;
use crate::ctrl::pin::ERR_PIN;

/// Universal ouput pin structure for platform independency
#[derive(Debug)]
pub struct UniOutPin {
    /// The pin control used
    #[cfg(feature = "rasp")]
    pub sys_pin : Option<OutputPin>,
    #[cfg(not(any(feature = "rasp")))]
    state : PinState, 

    /// The pin number
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
            impl UniOutPin {
                pub fn new(pin : u8) -> Self {
                    Self { 
                        pin,
                        state: PinState::Low
                    }
                }
            }

            impl Setup for UniOutPin { }

            impl Default for UniOutPin {
                fn default() -> Self {
                    Self {
                        pin: ERR_PIN,
                        state: PinState::Low
                    }
                }
            }

            #[cfg(not(any(feature = "rasp")))]
            impl embedded_hal::digital::v2::OutputPin for UniOutPin {
                type Error = ();

                fn set_high(&mut self) -> Result<(), Self::Error> {
                    self.state = PinState::High;
                    Ok(())
                }

                fn set_low(&mut self) -> Result<(), Self::Error> {
                    self.state = PinState::Low;
                    Ok(())
                }

                fn set_state(&mut self, state: PinState) -> Result<(), Self::Error> {
                    self.state = state;
                    Ok(())
                }
            }
        } 
    }
    //

    // Implementation for Raspberry Pi devices
    //  -> Implementation cannot fail
    cfg_if::cfg_if! { 
        if #[cfg(feature = "rasp")] {
            impl UniOutPin {
                pub fn new(pin : u8) -> Self {
                    Self {
                        pin,
                        sys_pin: None
                    }
                }
            }

            impl Default for UniOutPin {
                fn default() -> Self {
                    Self {
                        pin: ERR_PIN,
                        sys_pin: None
                    }
                }
            }

            impl Setup for UniOutPin {
                fn setup(&mut self) -> Result<(), crate::Error> {
                    self.sys_pin = Some(Gpio::new()?.get(self.pin)?.into_output());
                    Ok(())
                }
            }

            impl embedded_hal::digital::v2::OutputPin for UniOutPin {
                type Error = ();

                #[inline]
                fn set_high(&mut self) -> Result<(), Self::Error> {
                    if let Some(sys_pin) = &mut self.sys_pin {
                        Ok(sys_pin.set_high())
                    } else {
                        panic!("Pin not setup yet! (Number: {})", self.pin)
                    }
                }

                #[inline]
                fn set_low(&mut self) -> Result<(), Self::Error> {
                    if let Some(sys_pin) = &mut self.sys_pin {
                        Ok(sys_pin.set_low())
                    } else {
                        panic!("Pin not setup yet! (Number: {})", self.pin)
                    }
                }

                #[inline]
                fn set_state(&mut self, state: PinState) -> Result<(), Self::Error> {
                    match state {
                        PinState::Low => self.set_low(),
                        PinState::High => self.set_high()
                    }
                }
            }
        }
    }
    // 
// 