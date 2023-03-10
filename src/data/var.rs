use crate::units::*;

#[derive(Clone, Debug, Default)]
pub struct CompVars
{
    pub t_load : Force,
    pub j_load : Inertia
}

impl CompVars 
{
    pub const ZERO : Self = Self { t_load: Force::ZERO, j_load: Inertia::ZERO };
}