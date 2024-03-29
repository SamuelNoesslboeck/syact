// Submodules
// pub mod actors;
/// Iterators for different curve building application cases
#[allow(dead_code)]
mod builders;
pub use builders::*;

// /// Methods for calculating stepper motor acceleration curves
// pub mod curve;

/// Methods for calculating forces acting upon components and motors
pub mod force;
pub use force::{forces_joint, forces_segment};

/// Methods for calculating inertias of assemblies
pub mod inertia;

/// Functions describing general kinematic processes
pub mod kin;

/// Functions and structs for movements of single or multiple components
pub mod movements;

// /// Methods for calculating paths and movements
pub mod path;
