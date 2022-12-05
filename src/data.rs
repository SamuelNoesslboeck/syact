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

    /// Step count per revolution [in (1)]
    pub n_s : u64,
    /// Coil pair count (n_s / 2) in many cases [in (1)]
    pub n_c : u64,
    /// Stall current [in Nm]
    pub t_s : f32,
    /// Inhertia moment [in kg*m^2]
    pub j_s : f32,

    // Dynamic
    /// Load torque [in Nm]
    pub t_load : f32,   
    /// Load inertia [in kg*m^2]
    pub j_load : f32
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
            j_s: 0.000_005_7,

            t_load: 0.0,
            j_load: 0.0
        };
    }

    /// The maximum angular acceleration of the motor (in stall) [in s^-2]
    pub fn alpha_max(&self) -> f32 {
        self.t() / self.j()
    }

    pub fn alpha_max_dyn(&self, t_s : f32) -> f32 {
        self.t_dyn(t_s) / self.j()
    }

    /// The inductivity constant [in s]
    pub fn tau(&self) -> f32 {
        self.i_max * self.l / self.u
    }

    /// Time per step for the given omega [in s]
    pub fn time_step(&self, omega : f32) -> f32 {
        2.0 * PI / (self.n_s as f32) / omega
    }

    /// Omega for time per step [in 1/s]
    pub fn omega(&self, time_step : f32) -> f32 {
        (self.n_s as f32) / 2.0 / PI / time_step
    }

    /// Get the angular distance of a step in rad [in 1]
    pub fn step_ang(&self) -> f32 {
        2.0 * PI / self.n_s as f32
    }

    // Load calculations
        /// Max motor torque when having a load [in Nm]
        pub fn t(&self) -> f32 {
            (self.t_s - self.t_load).clamp(0.0, self.t_s)
        }

        pub fn t_dyn(&self, t_s : f32) -> f32 {
            (t_s - self.t_load).clamp(0.0, t_s)
        }

        /// Motor inertia when having a load [in kg*m^2]
        pub fn j(&self) -> f32 {
            self.j_s + self.j_load
        }
    //
}

pub struct SimpleServoData
{
    pub t_max : f32
}

// TODO: Servos