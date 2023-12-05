use crate::{ctrl::pin::{UniOutPin, UniInPin}, Setup};

pub type Switch = sylo::Switch<UniInPin>;
pub type Relay = sylo::Relay<UniOutPin>;

impl Setup for Relay {
    fn setup(&mut self) -> Result<(), crate::Error> {
        self.pin_mut().setup()
    }
}