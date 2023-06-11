use crate::{SyncComp, Setup};
use crate::ctrl::pin::{UniInPin, UniPin};
use crate::units::*;

use serde::{Serialize, Deserialize};

/// A structure for taking basic measurements
pub trait SimpleMeas : Setup {
    /// Measures the position given component and overwrites it's distance when it does so
    fn measure(&mut self, comp : &mut dyn SyncComp) -> Result<Delta, crate::Error>;
}

/// Defines basic measurement data used in a measurement process
pub trait MeasData {
    /// Returns a mutable reference to the main pin of the measurement 
    fn pin<'a>(&'a self) -> Option<&'a UniInPin>;

    /// Returns a mutable reference to the main pin of the measurement 
    fn pin_mut<'a>(&'a mut self) -> Option<&'a mut UniInPin>;
}

/// A simple endswitch that can trigger when reaching a destination
#[cfg_attr(feature = "std", derive(Serialize, Deserialize))]
pub struct EndSwitch {
    max_dist : Delta,
    set_val : Gamma,
    meas_speed_f : f32, 

    pin : u8,

    #[cfg_attr(feature = "std", serde(skip))]
    sys_pin : Option<UniInPin>
}

impl EndSwitch {
    /// Creates a new end switch
    pub fn new(pin : u8, max_dist : Delta, meas_speed_f : f32, set_val : Gamma) -> Self {
        Self {
            pin, max_dist, meas_speed_f, set_val,
            sys_pin: None
        }
    }
}

impl Setup for EndSwitch {
    fn setup(&mut self) -> Result<(), crate::Error> {
        self.sys_pin = Some(UniPin::new(self.pin)?.into_input());
        Ok(())
    }
}

impl SimpleMeas for EndSwitch {
    fn measure(&mut self, comp : &mut dyn SyncComp) -> Result<Delta, crate::Error> {
        let meas_res = comp.drive_rel_int(self.max_dist, self.meas_speed_f, |data| {
            let pin_opt = data.pin_mut(); 

            if let Some(pin) = pin_opt {
                pin.is_high()
            } else {
                false
            }
        }, self)?; 

        if meas_res.1 {
            comp.write_gamma(self.set_val);
            Ok(meas_res.0)
        } else {
            Err("Measurement failed! (Reached maximum distance)".into())
        }
    }
}

impl MeasData for EndSwitch {
    fn pin<'a>(&'a self) -> Option<&'a UniInPin> {
        if let Some(pin) = &self.sys_pin {
            Some(pin) 
        } else {
            None
        }
    }

    fn pin_mut<'a>(&'a mut self) -> Option<&'a mut UniInPin> {
        if let Some(pin) = &mut self.sys_pin {
            Some(pin) 
        } else {
            None
        }
    }
}

/// Struct that performs no measurement (placeholder)  \
/// Will be removed soon!
#[cfg_attr(feature = "std", derive(Serialize, Deserialize))]
pub struct NoMeas { }

impl Setup for NoMeas { }

impl SimpleMeas for NoMeas {
    fn measure(&mut self, _ : &mut dyn SyncComp) -> Result<Delta, crate::Error> {
        Ok(Delta::ZERO)
    }
}