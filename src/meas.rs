use crate::ctrl::{Interruptor, InterruptReason};
use crate::{units::*, SyncComp, Setup};

use serde::{Serialize, Deserialize};

// Submodules
    mod endswitch;
    pub use endswitch::*;
// 

/// A structure for taking basic measurements
pub trait SimpleMeas : Interruptor + Setup { 
    fn data(&self) -> &SimpleMeasData;
}

#[derive(Clone, Serialize, Deserialize)]
pub struct SimpleMeasData {
    pub set_gamma : Gamma,
    pub max_dist : Delta,

    pub meas_speed_f : f32,

    pub add_samples : usize,
    pub sample_dist : Delta
}

/// Simplest form of measurement by reference position
pub fn take_simple_meas<C : SyncComp + ?Sized>(comp : &mut C, data : &SimpleMeasData, speed_f : f32) -> Result<(), crate::Error> {
    let mut gammas : Vec<Gamma> = Vec::new();

    // Init measurement
        // Drive full distance with optionally reduced speed
        comp.drive_rel(data.max_dist, data.meas_speed_f * speed_f)?;

        // Check wheiter the component has been interrupted and if it is the correct interrupt
        if comp.intr_reason().ok_or("The measurement failed! No interrupt was triggered")? != InterruptReason::EndReached {
            return Err("Bad interrupt reason!".into());     // TODO: Improve error message
        }

        gammas.push(comp.gamma());
    //

    // Samples
        for _ in 0 .. data.add_samples {
            // Drive half of the sample distance back (faster)
            comp.drive_rel(-data.sample_dist / 2.0, speed_f)?;
            // Drive sample distance
            comp.drive_rel(data.sample_dist, data.meas_speed_f * speed_f)?;

            // Check wheiter the component has been interrupted and if it is the correct interrupt
            if comp.intr_reason().ok_or("The measurement failed! No interrupt was triggered")? != InterruptReason::EndReached {
                return Err("Bad interrupt reason!".into());     // TODO: Improve error message
            }

            gammas.push(comp.gamma());
        }
    // 

    // The average gamma of all measurements
    let gamma_av = Gamma(gammas.iter().map(|g| g.0).sum()) / (gammas.len() as f32);
    // Current gamma difference considering the current position and the average taken by the measurement
    let gamma_diff = comp.gamma() - gamma_av;
    // The new gamma to set the components gamma to
    let gamma_new = data.set_gamma + gamma_diff;

    // Set limits and write new distance value
    comp.set_end(gamma_av);
    comp.write_gamma(gamma_new);

    Ok(())
}