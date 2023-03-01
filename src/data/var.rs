use crate::{Force, Inertia};

#[derive(Clone, Debug, Default)]
pub struct StepperVar
{
    pub t_load : Force,
    pub j_load : Inertia
}

impl StepperVar 
{
    pub const ZERO : Self = Self { t_load: Force::ZERO, j_load: Inertia::ZERO };
}