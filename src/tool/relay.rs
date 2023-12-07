use crate::SimpleTool;
use crate::device::pin::{UniOutPin, UniPin};

/// A simple relay
pub struct Relay {
    pin : UniOutPin
}

impl Relay {
    /// Creates a new relay
    pub fn new(pin : u8) -> Result<Self, crate::Error> {
        Ok(Self {
            pin: UniPin::new(pin).map(|p| p.into_output())?
        })
    }
}

impl SimpleTool for Relay {
    fn activate(&mut self) {
        self.pin.set_high()
    }

    fn deactivate(&mut self) {
        self.pin.set_low()
    }

    fn is_active(&self) -> bool {
        self.pin.is_set_high()
    }
}