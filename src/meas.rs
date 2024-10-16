use alloc::vec::Vec;

use serde::{Serialize, Deserialize};
use syunit::*;

use crate::act::{ActuatorError, InterruptReason, Interruptible, SyncActuatorBlocking};

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
        SyncActuatorError(ActuatorError)
    }

    impl From<ActuatorError> for SimpleMeasError {
        fn from(value: ActuatorError) -> Self {
            SimpleMeasError::SyncActuatorError(value)
        }
    }
// 

/// Collection of parameters required for a simple measurement
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct SimpleMeasParams {
    /// The abs_pos value to set the component to if the measurement was successful
    pub overwrite_abs_pos : AbsPos,
    /// Maximum drive distance, also determines which direction will be used. 
    /// If the maximum distance is reached before the measurement is conducted, the measurement will count as failed
    pub max_dist : RelDist,

    /// Measurement speed factor (Optionally conduct a measurement slower)
    pub meas_speed : Factor,

    /// Number of additional samples to take
    _add_samples : Option<usize>,
    /// Will take 5% of max_dist as default
    pub sample_dist : Option<RelDist>
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

    /// Collection of all abs_pos values
    pub abs_poss : Vec<AbsPos>,
    /// Average abs_pos used for the set abs_pos
    pub abs_pos_av : AbsPos,
    /// Correction value (offset of current postion and set-abs_pos reference)
    pub corr : RelDist
}

impl SimpleMeasValues {
    /// Maximum abs_pos value measured
    pub fn abs_pos_max(&self) -> AbsPos {
        *self.abs_poss.iter().reduce(AbsPos::max_ref).expect("AbsPos array must contain a value")
    }

    /// Minimum abs_pos value measured
    pub fn abs_pos_min(&self) -> AbsPos {
        *self.abs_poss.iter().reduce(AbsPos::min_ref).expect("AbsPos array must contain a value")
    }

    /// Inaccuracy accross all abs_pos values measured
    pub fn max_inacc(&self) -> RelDist {
        self.abs_pos_max() - self.abs_pos_min()
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
pub fn take_simple_meas<C : SyncActuatorBlocking + Interruptible + ?Sized>(comp : &mut C, data : &SimpleMeasParams, speed : Factor) -> Result<SimpleMeasValues, SimpleMeasError> {
    let mut abs_poss : Vec<AbsPos> = Vec::new();

    // Init measurement
        // Drive full distance with optionally reduced speed
        comp.drive_rel(data.max_dist, data.meas_speed * speed)?;
        
        comp.intr_reason()      // Get the interrupt reason
            .ok_or(SimpleMeasError::NoInterrupt)
            .and_then(|reason|
                if reason == InterruptReason::EndReached { Ok(()) } else { Err(SimpleMeasError::WrongInterruptReason(reason)) }
            )?;       // If no interrupt was triggered, return `MeasError::NoInterrupt`

        abs_poss.push(comp.abs_pos());
    //

    // Samples
        for _ in 0 .. data.add_samples() {
            // Drive half of the sample distance back (faster)
            comp.drive_rel(-data.sample_dist.unwrap_or(data.max_dist * 0.25) / 2.0, speed)?;

            // TODO: Check for errors when moving backwards

            // Drive sample distance
            comp.drive_rel(data.sample_dist.unwrap_or(data.max_dist * 0.25), data.meas_speed * speed)?;

            // Check wheiter the component has been interrupted and if it is the correct interrupt
            comp.intr_reason()      // Get the interrupt reason
                .ok_or(SimpleMeasError::NoInterrupt)
                .and_then(|reason|
                    if reason == InterruptReason::EndReached { Ok(()) } else { Err(SimpleMeasError::WrongInterruptReason(reason)) }
                )?;       // If no interrupt was triggered, return `MeasError::NoInterrupt`

            // Add the measurement value to the list
            abs_poss.push(comp.abs_pos());
        }
    // 

    // The average abs_pos of all measurements
    let abs_pos_av = AbsPos(abs_poss.iter().map(|g| g.0).sum()) / (abs_poss.len() as f32);
    // Current abs_pos difference considering the current position and the average taken by the measurement
    let abs_pos_diff = comp.abs_pos() - abs_pos_av;
    // The new abs_pos to set the components abs_pos to
    let abs_pos_new = data.overwrite_abs_pos + abs_pos_diff;

    // Set limits and write new distance value
    comp.set_endpos(abs_pos_av);
    comp.overwrite_abs_pos(abs_pos_new);

    Ok(SimpleMeasValues {
        samples: data.add_samples(),

        abs_poss: abs_poss,
        abs_pos_av: abs_pos_av,
        corr: abs_pos_diff
    })
}