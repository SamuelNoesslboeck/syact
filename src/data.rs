use std::f32::consts::PI;

/// A collection of the most relevant variables in stepper calculation
pub struct StepperData
{
    /// Motor voltage [in V]
    pub u : f32,
    /// Max phase current [in A]
    pub i_max : f32,
    /// Motor inductence [in H]
    pub l : f32,

    /// Step count [in (1)]
    pub n_s : u64,
    /// Charge count [in (1)]
    pub n_c : u64,
    /// Stall current [in Nm]
    pub t_s : f32,
    /// Inhertia moment [in kg*m^2]
    pub j : f32
}

impl StepperData
{
    /// Stepper motor from stepperonline 
    /// Link: https://www.omc-stepperonline.com/de/e-serie-nema-17-bipolar-42ncm-59-49oz-in-1-5a-42x42x38mm-4-draehte-w-1m-kabel-verbinder-17he15-1504s
    pub fn mot_17he15_1504s(u : f32) -> Self {
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
    pub fn alpha_max(&self) -> f32 {
        return self.t_s / self.j;
    }

    /// The inductivity constant [in s]
    pub fn tau(&self) -> f32 {
        return self.i_max * self.l / self.u;
    }

    /// Time per step for the given omega
    pub fn time_step(&self, omega : f32) -> f32 {
        return 2.0 * PI / (self.n_s as f32) / omega;
    }

    /// Omega for time per step
    pub fn omega(&self, time_step : f32) -> f32 {
        return (self.n_s as f32) / 2.0 / PI / time_step;
    }

    /// Get the angular distance of a step in rad
    pub fn ang_dis(&self) -> f32 {
        return 2.0 * PI / self.n_s as f32;
    }
}