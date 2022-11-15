use crate::{StepperData, controller::StepperCtrl, UpdateFunc};

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

    pub fn write_length(&mut self, dis : f32) {
        self.ctrl.write_pos(self.phi_c(dis));
    }

    /// Returns the extension of the cylinder
    pub fn length(&self) -> f32 {
        return self.dis_c(self.ctrl.get_abs_pos());
    }
}

pub struct CylinderTriangle 
{
    // Triangle
    pub l_a : f32,
    pub l_b : f32,

    // Cylinder length
    pub cylinder : Cylinder
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

    pub fn length_for_gamma(&self, gam : f32) -> f32 {
        (self.l_a.powi(2) + self.l_b.powi(2) + 2.0 * self.l_a * self.l_b * gam.cos()).powf(0.5)
    }

    pub fn get_gamma(&self) -> f32 {
        ((self.cylinder.length().powi(2) as f32 - self.l_a.powi(2) - self.l_b.powi(2)) / 2.0 / self.l_a / self.l_b).acos()
    }

    pub fn set_gamma(&mut self, gam : f32, v_max : f32) {
        self.cylinder.extend(self.length_for_gamma(gam), v_max);
    }

    // pub fn write_gamma(&mut self, gam : f32) {
        
    // }
}

pub struct GearBearing 
{
    pub ctrl : Box<dyn StepperCtrl>,
    
    pub ratio : f32
}

impl GearBearing 
{
    pub fn get_pos(&self) -> f32 {
        self.ctrl.get_abs_pos() * self.ratio
    }

    pub fn set_pos(&mut self, pos : f32, omega : f32) -> f32 {
        self.ctrl.drive((pos - self.get_pos()) / self.ratio, omega, UpdateFunc::None)
    }
}