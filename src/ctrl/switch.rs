use crate::ctrl::pin::{UniOutPin, UniInPin};

pub type Switch = sylo::Switch<UniInPin>;
pub type Relay = sylo::Relay<UniOutPin>;