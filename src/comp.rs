use crate::{StepperData, ctrl::StepperCtrl, UpdateFunc, Vec3};

pub struct Cylinder
{
    /// Data of the connected stepper motor
    pub ctrl : Box<dyn StepperCtrl>,

    /// Distance traveled per rad [in mm]
    pub rte_ratio : f32,
    
    /// Minimal extent [in mm]
    pub pos_min : f32,
    /// Maximal extent [in mm]
    pub pos_max : f32
}

impl Cylinder
{
    /// Create a new cylinder instance
    pub fn new(ctrl : Box<dyn StepperCtrl>, rte_ratio : f32, pos_min : f32, pos_max : f32) -> Self {
        return Cylinder {
            ctrl,
            rte_ratio,
            pos_min,        // TODO: Use min and max
            pos_max
        };
    }

    /// Get the stepper motor data for the cylinder
    pub fn data(&self) -> &StepperData {
        self.ctrl.get_data()
    }

    // Conversions
        /// Angle for extent
        pub fn phi_c(&self, dis_c : f32) -> f32 {
            dis_c / self.rte_ratio
        }

        /// Extent for angle
        pub fn dis_c(&self, phi_c : f32) -> f32 {
            phi_c * self.rte_ratio
        }

        /// Angular speed for linear velocity
        pub fn omega_c(&self, v_c : f32) -> f32 {
            v_c / self.rte_ratio
        }

        /// Linear velocity for angular speed
        pub fn v_c(&self, omega : f32) -> f32 {
            omega * self.rte_ratio
        }
    //
    
    /// Extend the cylinder by a given distance _dis_ (in mm) with the maximum velocity _v max_ (in mm/s), returns the actual distance traveled
    pub fn extend(&mut self, dis : f32, v_max : f32) -> f32 {
        self.ctrl.drive(self.phi_c(dis), self.omega_c(v_max), UpdateFunc::None)
    }

    pub fn measure(&mut self, max_dis : f32, v_max : f32, dir : bool, set_len : f32, accuracy : u64) {
        self.ctrl.measure(
            self.ctrl.ang_to_steps(self.phi_c(max_dis)), 
            self.omega_c(v_max), 
            dir, 
            self.ctrl.ang_to_steps_dir(self.phi_c(set_len)), 
            accuracy
        );
    }

    /// Overwrite the current cylinder length without moving
    pub fn write_length(&mut self, dis_c : f32) {
        self.ctrl.write_pos(self.phi_c(dis_c));
    }

    /// Returns the length of the cylinder
    pub fn length(&self) -> f32 {
        return self.dis_c(self.ctrl.get_abs_pos());
    }

    // Loads
    pub fn apply_load_m(&mut self, mass : f32) {
        self.ctrl.apply_load_j(mass * self.rte_ratio.powi(2));
    }

    pub fn apply_load_f(&mut self, force : f32) {
        self.ctrl.apply_load_t(force * self.rte_ratio);
    }
    //
}

pub struct CylinderTriangle 
{
    // Cylinder
    pub cylinder : Cylinder,

    // Triangle
    pub l_a : f32,
    pub l_b : f32
}

impl CylinderTriangle 
{
    pub fn new(cylinder : Cylinder, l_a : f32, l_b : f32) -> Self
    {
        let mut tri = CylinderTriangle {
            l_a, 
            l_b,
            cylinder 
        };

        tri.cylinder.write_length(l_a.max(l_b));

        return tri;
    }

    /// Returns the cylinder length for the given angle gamma
    pub fn length_for_gamma(&self, gam : f32) -> f32 {
        (self.l_a.powi(2) + self.l_b.powi(2) + 2.0 * self.l_a * self.l_b * gam.cos()).powf(0.5)
    }

    /// Returns the angle gamma for the given cylinder length _(len < (l_a + l_b))_
    pub fn gamma_for_length(&self, len : f32) -> f32 {
        ((len.powi(2) as f32 - self.l_a.powi(2) - self.l_b.powi(2)) / 2.0 / self.l_a / self.l_b).acos()
    }

    // Angle
        pub fn get_gamma(&self) -> f32 {
            self.gamma_for_length(self.cylinder.length())
        }

        pub fn set_gamma(&mut self, gam : f32, v_max : f32) {
            self.cylinder.extend(self.length_for_gamma(gam), v_max);
        }
    //

    // pub fn write_gamma(&mut self, gam : f32) {
        
    // }
}

/// A bearing powered by a motor with a certain gear ratio
pub struct GearBearing 
{
    /// Steppercontrol for the motor of the bearing
    pub ctrl : Box<dyn StepperCtrl>,
    
    /// Angle ration from motor to bearing (omega_b / omega_m)
    pub ratio : f32
}

impl GearBearing 
{
    // Converstions
        /// Returns the angle for the motor from a given bearing angle
        pub fn ang_for_motor(&self, ang : f32) -> f32 {
            ang / self.ratio
        }

        /// Returns the omega for the motor from a given bearing omega
        pub fn omega_for_motor(&self, omega : f32) -> f32 {
            omega / self.ratio
        }
    //  

    pub fn get_pos(&self) -> f32 {
        self.ctrl.get_abs_pos() * self.ratio
    }

    pub fn set_pos(&mut self, pos : f32, omega : f32) -> f32 {
        self.ctrl.drive(self.ang_for_motor(pos - self.get_pos()), self.omega_for_motor(omega), UpdateFunc::None)
    }

    pub fn measure(&mut self, max_angle : f32, omega : f32, dir : bool, set_pos : f32, accuracy : u64) {
        self.ctrl.measure(
            self.ctrl.ang_to_steps(self.ang_for_motor(max_angle)), 
            self.omega_for_motor(omega), 
            dir, 
            self.ctrl.ang_to_steps_dir(self.ang_for_motor(set_pos)),
            accuracy
        );
    }
}

// Tools
pub trait Tool
{
    /// Returns the characteristic vector of the tool
    fn get_vec(&self) -> Vec3;
}

pub struct NoTool { } // Empty struct

impl NoTool 
{
    pub fn new() -> Self {
        Self { }
    }
}

impl Tool for NoTool
{
    fn get_vec(&self) -> Vec3 {
        Vec3::new(0.0, 0.0, 0.0)
    }
}