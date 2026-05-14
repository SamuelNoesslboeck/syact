use alloc::vec::Vec;

#[cfg(feature = "serde")]
use serde::{Serialize, Deserialize};
use syunit::*;

use crate::{ActuatorError, InterruptReason, Interruptible, SyncActuator};

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
    #[derive(Debug)]
    pub enum SimpleMeasError<U : UnitSet> {
        /// There was no interrupt triggered while driving, meaning that either
        /// - the interrupt source is out of reach (e.g. endstop is not close enough)
        /// - the interrupt source is broken, not correctly wired or similar
        NoInterrupt,
        /// The motor stopped because of the wrong reason, e.g. overloading it
        WrongInterruptReason(InterruptReason),
        /// There was an issue with the motor itself
        SyncActuatorError(ActuatorError<U>)
    }

    impl<U : UnitSet> From<ActuatorError<U>> for SimpleMeasError<U> {
        fn from(value: ActuatorError<U>) -> Self {
            SimpleMeasError::SyncActuatorError(value)
        }
    }
// 

/// Collection of parameters required for a simple measurement
#[derive(Debug, Clone, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct SimpleMeasParams<U : UnitSet> {
    /// The pos value to set the component to if the measurement was successful
    pub new_abs_pos : U::Position,
    /// Maximum drive distance, also determines which direction will be used. 
    /// If the maximum distance is reached before the measurement is conducted, the measurement will count as failed
    pub max_dist : U::Distance,

    /// Measurement speed factor (Optionally conduct a measurement slower)
    pub meas_speed : Factor,
    
    /* Optionals */
        /// Number of additional samples to take
        _add_samples : Option<usize>,
        /// Distance the component drives back to take additional samples
        _sample_dist : Option<U::Distance>,
        /// Whether or not the limits should be set after a successful measurement
        pub set_limit : bool
    /**/
}

impl<U : UnitSet> SimpleMeasParams<U> {
    pub const fn new(new_abs_pos : U::Position, max_dist : U::Distance, meas_speed : Factor) -> Self {
        Self {
            new_abs_pos,
            max_dist,
            meas_speed,

            _add_samples: None,
            _sample_dist: None,
            set_limit: false
        }
    }

    /// Additional samples to be taken for accuracy
    pub const fn with_add_samples(mut self, count : usize) -> Self {
        self._add_samples = Some(count);
        self
    }

    /// The distance to be used for samples
    pub const fn with_sample_distance(mut self, dist : U::Distance) -> Self {
        // TODO: Add check for valid distance
        
        self._sample_dist = Some(dist);
        self
    }

    pub const fn with_setting_limit(mut self) -> Self {
        self.set_limit = true;
        self
    }

    /// Number of additional samples to take, defaulting to 0
    pub fn add_samples(&self) -> usize {
        self._add_samples.unwrap_or(0)
    }

    /// Distance the component drives back to take additional samples, defaulting to 10% of max distance
    pub fn sample_dist(&self) -> U::Distance {
        self._sample_dist.unwrap_or(
            self.max_dist * 0.1
        )
    }
}

/// Result of a simple measurement
#[derive(Debug, Clone, Default)]
pub struct SimpleMeasValues<U : UnitSet> {
    /// Number of samples taken
    pub sample_count : usize,

    /// Collection of all pos values
    pub positions : Vec<U::Position>,
    /// Average pos used for the set pos
    pub position_avg : U::Position,
    /// Correction value (offset of current postion and set-pos reference)
    pub correction : U::Distance
}

impl<U : UnitSet> SimpleMeasValues<U> {
    /// Maximum pos value measured
    pub fn abs_pos_max(&self) -> U::Position {
        *self.positions.iter().reduce(U::Position::max_ref).expect("Position array must contain a value")
    }

    /// Minimum pos value measured
    pub fn abs_pos_min(&self) -> U::Position {
        *self.positions.iter().reduce(U::Position::min_ref).expect("Position array must contain a value")
    }

    /// Inaccuracy accross all pos values measured
    pub fn max_inacc(&self) -> U::Distance {
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
pub async fn take_simple_meas<U : UnitSet, C : SyncActuator<U> + Interruptible + ?Sized>(comp : &mut C, params : &SimpleMeasParams<U>, speed : Factor) -> Result<SimpleMeasValues<U>, SimpleMeasError<U>> {
    let mut abs_poss : Vec<U::Position> = Vec::new();

    // Init measurement
        // Drive full distance with optionally reduced speed
        comp.drive_rel(params.max_dist, params.meas_speed * speed).await?;
        
        // Check whether the component has been interrupted and if it is the correct interrupt
        comp.intr_reason()      // Get the interrupt reason
            .ok_or(SimpleMeasError::NoInterrupt)
            .and_then(|reason|
                if reason == InterruptReason::HardwareEndReached { 
                    Ok(())  // Let hardware interrupts pass for measurements
                } else { 
                    Err(SimpleMeasError::WrongInterruptReason(reason))  
                }
            )?;       // If no interrupt was triggered, return `MeasError::NoInterrupt`    

        abs_poss.push(comp.pos());
    //

    // Additional samples
        for _ in 0 .. params.add_samples() {
            // Drive half of the sample distance back (faster)
            comp.drive_rel(-params.sample_dist(), speed).await?;

            // TODO: Check for errors when moving backwards

            // Drive sample distance
            comp.drive_rel(params.sample_dist() * 1.5, params.meas_speed * speed).await?;

            // Check whether the component has been interrupted and if it is the correct interrupt
            comp.intr_reason()      // Get the interrupt reason
                .ok_or(SimpleMeasError::NoInterrupt)
                .and_then(|reason|
                    if reason == InterruptReason::HardwareEndReached { 
                        Ok(())  // Let hardware interrupts pass for measurements
                    } else { 
                        Err(SimpleMeasError::WrongInterruptReason(reason))  
                    }
                )?;       // If no interrupt was triggered, return `MeasError::NoInterrupt`

            // Add the measurement value to the list
            abs_poss.push(comp.pos());
        }
    // 

    // The average pos of all measurements
    let abs_pos_av = U::Position::from(abs_poss.iter().map(|g| (*g).into()).sum()) / (abs_poss.len() as f32);
    // Current pos difference considering the current position and the average taken by the measurement
    let abs_pos_diff = comp.pos() - abs_pos_av;
    // The new pos to set the components pos to
    let abs_pos_new = params.new_abs_pos + abs_pos_diff;

    // Set limits and write new distance value
    comp.overwrite_abs_pos(abs_pos_new);

    if params.set_limit {
        comp.set_endpos(abs_pos_av);
    }

    Ok(SimpleMeasValues {
        sample_count: params.add_samples() + 1,

        positions: abs_poss,
        position_avg: abs_pos_av,
        correction: abs_pos_diff
    })
}