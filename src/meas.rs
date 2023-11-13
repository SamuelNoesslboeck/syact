use crate::ctrl::InterruptReason;
use crate::SyncComp;
use crate::units::*;

use serde::{Serialize, Deserialize};

// Public imports
pub use sylo::{BoolMeas, ByteMeas, ShortMeas, IntMeas};

// Submodules
    mod endswitch;
    pub use endswitch::*;

    mod sonar;
    pub use sonar::*;
// 

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SimpleMeasData {
    pub set_gamma : Gamma,
    pub max_dist : Delta,

    pub meas_speed_f : f32,

    #[serde(default = "default_add_samples")]
    pub add_samples : usize,
    /// Will take 5% of max_dist as default
    pub sample_dist : Option<Delta>
}

// Defaults
    /// The default number of measurement samples to take
    const fn default_add_samples() -> usize { 1 }
// 

#[derive(Debug, Clone)]
pub struct SimpleMeasResult {
    pub samples : usize,

    // Gamma values
    pub gammas : Vec<Gamma>,
    pub gamma_av : Gamma,
    pub corr : Delta
}

impl SimpleMeasResult {
    pub fn gamma_max(&self) -> Gamma {
        *self.gammas.iter().reduce(Gamma::max_ref).expect("Gamma array must contain a value")
    }

    pub fn gamma_min(&self) -> Gamma {
        *self.gammas.iter().reduce(Gamma::min_ref).expect("Gamma array must contain a value")
    }

    pub fn max_inacc(&self) -> Delta {
        self.gamma_max() - self.gamma_min()
    }
}

/// Simplest form of measurement by reference position
/// - `comp`: The component to measure
/// - `data`: The data defining the measurement
/// - `speed_f`: The overall speed factor of the measurement
/// 
/// # Measurement data and its usage
/// 
/// Specifing a `sample_dist` is optional, as the script will replace it with 10% of the maximum distance if not specified
pub fn take_simple_meas<C : SyncComp + ?Sized>(comp : &mut C, data : &SimpleMeasData, speed_f : f32) -> Result<SimpleMeasResult, crate::Error> {
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
            println!("- Gamma: {}", comp.gamma());

            // Drive half of the sample distance back (faster)
            comp.drive_rel(-data.sample_dist.unwrap_or(data.max_dist * 0.1) / 2.0, speed_f)?;

            println!("- Gamma: {}", comp.gamma());

            // Drive sample distance
            comp.drive_rel(data.sample_dist.unwrap_or(data.max_dist * 0.1), data.meas_speed_f * speed_f)?;

            println!("- Gamma: {}", comp.gamma());

            // Check wheiter the component has been interrupted and if it is the correct interrupt
            if comp.intr_reason().ok_or("The measurement failed! No interrupt was triggered")? != InterruptReason::EndReached {
                return Err("Bad interrupt reason!".into());     // TODO: Improve error message
            }

            // Add the measurement value to the list
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

    Ok(SimpleMeasResult {
        samples: data.add_samples,

        gammas: gammas,
        gamma_av: gamma_av,
        corr: gamma_diff
    })
}