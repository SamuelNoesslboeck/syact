use std::f64::consts::PI;

/// A collection of the most relevant variables in stepper calculation
pub struct StepperData
{
    /// Motor voltage [in V]
    pub u : f64,
    /// Max phase current [in A]
    pub i_max : f64,
    /// Motor inductence [in H]
    pub l : f64,

    /// Step count [in (1)]
    pub n_s : u64,
    /// Charge count [in (1)]
    pub n_c : u64,
    /// Stall current [in Nm]
    pub t_s : f64,
    /// Inhertia moment [in kg*m^2]
    pub j : f64
}

impl StepperData
{
    /// Stepper motor from stepperonline 
    /// Link: https://www.omc-stepperonline.com/de/e-serie-nema-17-bipolar-42ncm-59-49oz-in-1-5a-42x42x38mm-4-draehte-w-1m-kabel-verbinder-17he15-1504s
    pub fn mot_17he15_1504s(u : f64) -> Self {
        return StepperData { 
            u, 
            i_max: 1.5, 
            l: 0.004, 
            n_s: 200, 
            n_c: 100,
            t_s: 0.42, 
            j: 0.000_005_7 
        };
    }

    /// The maximum angular acceleration of the motor (in stall) [in s^-2]
    pub fn alpha_max(&self) -> f64 {
        return self.t_s / self.j;
    }

    /// The inductivity constant [in s]
    pub fn tau(&self) -> f64 {
        return self.i_max * self.l / self.u;
    }

    /// Time per step for the given omega
    pub fn time_step(&self, omega : f64) -> f64 {
        return 2.0 * PI / (self.n_s as f64) / omega;
    }

    /// Omega for time per step
    pub fn omega(&self, time_step : f64) -> f64 {
        return (self.n_s as f64) / 2.0 / PI / time_step;
    }
}

pub struct Cylinder
{
    pub data : StepperData,

    pub pos : f64,
    
    pub pos_min : f64,
    pub pos_max : f64
}

impl Cylinder
{
    
}