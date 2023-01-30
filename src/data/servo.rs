use super::*; 

pub struct ServoData
{
    /// Maximum torque of servo motor [Unit Nm]
    pub t_max : f32,

    /// Maximum angular velocity [Unit rad/s]
    pub omega_max : f32,

    // Maximum angle [Unit (radians)]
    pub phi_max : f32,

    /// Minimum signal length [Unit s]
    pub pwm_min : f32,
    /// Maximum signal length [Unit s]
    pub pwm_max : f32,

    /// Default frequency
    pub f_pwm : f32
}

impl ServoData 
{
    pub fn mg996r() -> Self {
        ServoData {
            t_max: 1.08,
            omega_max: 8.5,
            phi_max: PI,

            pwm_min: 0.001,
            pwm_max: 0.002,
            f_pwm: 50.0
        }
    }

    pub fn default(&self) -> f32 {
        (self.pwm_min + self.phi_max) / 2.0
    }

    pub fn default_pos(&self) -> f32 {
        self.phi_max / 2.0
    }

    pub fn cycle_time(&self) -> f32 {
        1.0 / self.f_pwm
    }

    pub fn pulse_time(&self, phi : f32) -> f32 {
        self.pwm_min + (self.pwm_max - self.pwm_min) / self.phi_max * phi
    }
}

// TODO: Servos