use serde::{Serialize, Deserialize};
use syunit::*;

use crate::SyncActuator;
use crate::act::{InterruptReason, Interruptible, SyncActuatorError};

// Submodules
    mod endstop;
    pub use endstop::*;
// 

// Traits
    /// Traits for objects that can conduct a measurement and return a value 
    /// 
    /// # Generic `V`
    /// 
    /// Data type that will be returned by the measurement
    pub trait Measurable<V> {
        /// Error that can occur when measuring
        type Error;

        /// Conduct the measurement 
        fn measure(&mut self) -> Result<V, Self::Error>;
    }
// 

// Errors
    /// Error that can occur when using simple measurements
    pub enum SimpleMeasError {
        /// There was no interrupt triggered while driving, meaning that either
        /// - the interrupt source is out of reach (e.g. endstop is not close enough)
        /// - the interrupt source is broken, not correctly wired or similar
        NoInterrupt,
        /// The motor stopped because of the wrong reason, e.g. overloading it
        WrongInterruptReason(InterruptReason),
        /// There was an issue with the motor itself
        SyncActuatorError(SyncActuatorError)
    }

    impl From<SyncActuatorError> for SimpleMeasError {
        fn from(value: SyncActuatorError) -> Self {
            SimpleMeasError::SyncActuatorError(value)
        }
    }
// 

/// Collection of parameters required for a simple measurement
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct SimpleMeasParams {
    /// The gamma value to set the component to if the measurement was successful
    pub set_gamma : Gamma,
    /// Maximum drive distance, also determines which direction will be used. 
    /// If the maximum distance is reached before the measurement is conducted, the measurement will count as failed
    pub max_dist : Delta,

    /// Measurement speed factor (Optionally conduct a measurement slower)
    pub meas_speed : Factor,

    /// Number of additional samples to take
    _add_samples : Option<usize>,
    /// Will take 5% of max_dist as default
    pub sample_dist : Option<Delta>
}

impl SimpleMeasParams {
    /// Number of additional samples to take
    pub fn add_samples(&self) -> usize {
        self._add_samples.unwrap_or(1)
    }
}

/// Result of a simple measurement
#[derive(Debug, Clone, Default)]
pub struct SimpleMeasValues {
    /// Number of samples taken
    pub samples : usize,

    /// Collection of all gamma values
    pub gammas : Vec<Gamma>,
    /// Average gamma used for the set gamma
    pub gamma_av : Gamma,
    /// Correction value (offset of current postion and set-gamma reference)
    pub corr : Delta
}

impl SimpleMeasValues {
    /// Maximum gamma value measured
    pub fn gamma_max(&self) -> Gamma {
        *self.gammas.iter().reduce(Gamma::max_ref).expect("Gamma array must contain a value")
    }

    /// Minimum gamma value measured
    pub fn gamma_min(&self) -> Gamma {
        *self.gammas.iter().reduce(Gamma::min_ref).expect("Gamma array must contain a value")
    }

    /// Inaccuracy accross all gamma values measured
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
pub async fn take_simple_meas<C : SyncActuator + Interruptible + ?Sized>(comp : &mut C, data : &SimpleMeasParams, speed : Factor) -> Result<SimpleMeasValues, SimpleMeasError> {
    let mut gammas : Vec<Gamma> = Vec::new();

    // Init measurement
        // Drive full distance with optionally reduced speed
        comp.drive_rel(data.max_dist, data.meas_speed * speed).await?;
        
        comp.intr_reason()      // Get the interrupt reason
            .ok_or(SimpleMeasError::NoInterrupt)
            .and_then(|reason|
                if reason == InterruptReason::EndReached { Ok(()) } else { Err(SimpleMeasError::WrongInterruptReason(reason)) }
            )?;       // If no interrupt was triggered, return `MeasError::NoInterrupt`

        gammas.push(comp.gamma());
    //

    // Samples
        for _ in 0 .. data.add_samples() {
            // Drive half of the sample distance back (faster)
            comp.drive_rel(-data.sample_dist.unwrap_or(data.max_dist * 0.25) / 2.0, speed).await?;

            // TODO: Check for errors when moving backwards

            // Drive sample distance
            comp.drive_rel(data.sample_dist.unwrap_or(data.max_dist * 0.25), data.meas_speed * speed).await?;

            // Check wheiter the component has been interrupted and if it is the correct interrupt
            comp.intr_reason()      // Get the interrupt reason
                .ok_or(SimpleMeasError::NoInterrupt)
                .and_then(|reason|
                    if reason == InterruptReason::EndReached { Ok(()) } else { Err(SimpleMeasError::WrongInterruptReason(reason)) }
                )?;       // If no interrupt was triggered, return `MeasError::NoInterrupt`

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
    comp.set_endpos(gamma_av);
    comp.set_gamma(gamma_new);

    Ok(SimpleMeasValues {
        samples: data.add_samples(),

        gammas: gammas,
        gamma_av: gamma_av,
        corr: gamma_diff
    })
}