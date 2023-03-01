extern crate alloc;
use alloc::sync::Arc;

use serde::{Serialize, Deserialize};

use crate::{Component, LinkedData, Gamma, Omega, Delta, Alpha, Force, Inertia};
use crate::comp::{Cylinder, SimpleMeas};
use crate::math::MathActor;

/// A component representing a cylinder connected to two segments with constant lengths, forming a triangular shape
/// 
/// # Super Component
/// 
/// Uses a [Cylinder] as super component and as the triangular shape makes a constant angular velocity
/// very calculation expensive, all maximum velocites are referencing the cylinder
/// 
/// # Angles and lengths
/// 
/// The struct uses the default labeling of a mathematical triangles with the sides a, b and c. Here, a and b are the 
/// constant lengths and c being the variable cylinder length. All angles used are alpha, beta and gamma, all depending 
/// on the variable length c, making them also variable. The most relevant angle being gamma, as it is the opposing angle to 
/// the length c, representing the 
#[derive(Debug, Serialize, Deserialize)]
pub struct CylinderTriangle 
{
    /// The cylinder of the triangle, being the *super component* for this one
    pub cylinder : Cylinder,

    // Triangle
    /// The constant length of the first triangle component in millimeters
    pub l_a : f32,
    /// The constant length of the second triangle component in millimeters
    pub l_b : f32,
}

impl CylinderTriangle 
{
    /// Creates a new instance of a [CylinderTriangle], 
    /// writing an initial length of the longer segments the cylinder, preventing initial calculation errors
    pub fn new(cylinder : Cylinder, l_a : f32, l_b : f32) -> Self
    {
        let mut tri = CylinderTriangle {
            l_a, 
            l_b,

            cylinder
        };

        tri.cylinder.write_gamma(Gamma(l_a.max(l_b)));

        return tri;
    }

    // Conversions
        /// Returns the alpha angle (opposing to the a-segment) for a given gamma angle `gam`
        pub fn alpha_for_gam(&self, gam : Gamma) -> f32 {
            (self.l_a / self.gamma_for_super(gam).0 * gam.sin()).asin()
        }

        /// Returns the beta angle (opposing to the b-segment) for a given gamma angle `gam`
        pub fn beta_for_gam(&self, gam : Gamma) -> f32 {
            (self.l_b / self.gamma_for_super(gam).0 * gam.sin()).asin()
        }  

        /// Converts the given linear velocity `vel` to the angluar velocity for the given gamma angle `gam`
        pub fn omega_for_gam(&self, omega : Omega, gam : Gamma) -> Omega {
            omega / self.l_a * self.beta_for_gam(gam).sin()
        }

        /// Converts the given angular velocity `vel` to the linear velocity for the given gamma angle `gam`
        pub fn vel_for_gam(&self, omega : Omega, gam : Gamma) -> Omega {
            omega * self.l_a / self.beta_for_gam(gam).sin()
        }
    //
}

impl MathActor for CylinderTriangle
{
    fn accel_dyn(&self, omega : Omega, gamma : Gamma) -> Alpha {
        self.alpha_for_this(self.cylinder.accel_dyn(self.omega_for_super(omega, gamma), self.gamma_for_super(gamma)), self.gamma_for_super(gamma))
    }
}

impl SimpleMeas for CylinderTriangle
{
    fn init_meas(&mut self, pin_meas : u16) {
        self.cylinder.init_meas(pin_meas)
    }
}

impl Component for CylinderTriangle {
    // Data
        fn consts(&self) -> crate::StepperConst {
            self.cylinder.consts()
        }
    // 

    // Super 
        fn super_comp(&self) -> Option<&dyn Component> {
            Some(&self.cylinder)
        }

        fn super_comp_mut(&mut self) -> Option<&mut dyn Component> {
            Some(&mut self.cylinder)
        }

        /// Returns the cylinder length for the given angle gamma
        fn gamma_for_super(&self, gam : Gamma) -> Gamma {
            Gamma((self.l_a.powi(2) + self.l_b.powi(2) - 2.0 * self.l_a * self.l_b * gam.0.cos()).powf(0.5))
        }

        /// Returns the angle gamma for the given cylinder length _(len < (l_a + l_b))_
        fn gamma_for_this(&self, len : Gamma) -> Gamma {
            Gamma(((self.l_a.powi(2) + self.l_b.powi(2) - len.0.powi(2)) / 2.0 / self.l_a / self.l_b).acos())
        }

        fn omega_for_this(&self, super_omega : Omega, this_gamma : Gamma) -> Omega {
            super_omega / self.l_a * self.beta_for_gam(this_gamma).sin()
        }
    // 

    // Link
        fn link(&mut self, lk : Arc<LinkedData>) {
            self.cylinder.link(lk);
        }
    //

    // JSON I/O
        fn to_json(&self) -> Result<serde_json::Value, serde_json::Error> {
            serde_json::to_value(self)
        }
    //

    /// See [Component::drive()](`Component::drive()`)
    /// - `dist`is the angular distance to be moved (Unit radians)
    /// - `vel` is the cylinders extend velocity (Unit mm per second)
    fn drive_rel(&mut self, delta : Delta, omega : Omega) -> Gamma {
        self.cylinder.drive_rel(self.gamma_for_super(delta + self.get_gamma()) - self.cylinder.get_gamma(), omega)
    }

    /// See [Component::drive_async()](`Component::drive_async()`)
    /// - `dist`is the angular distance to be moved (Unit radians)
    /// - `vel` is the cylinders extend velocity (Unit mm per second)
    fn drive_rel_async(&mut self, delta : Delta, omega : Omega) {
        self.cylinder.drive_rel_async(self.gamma_for_super(delta + self.get_gamma()) - self.cylinder.get_gamma(), omega)
    }

    /// See [Component::drive_abs](`Component::drive_abs()`)
    /// - `dist`is the angular distance to be moved (Unit radians)
    /// - `vel` is the cylinders extend velocity (Unit mm per second)
    fn drive_abs(&mut self, gamma : Gamma, omega : Omega) -> Gamma {
        self.cylinder.drive_abs(self.gamma_for_super(gamma), omega)
    }

    fn drive_abs_async(&mut self, gamma : Gamma, omega : Omega) {
        self.cylinder.drive_abs_async(self.gamma_for_super(gamma), omega)
    }

    /// See [Component::measure()](`Component::measure()`)
    /// - `dist` is the maximum distance for the cylinder in mm
    /// - `vel` is the maximum linear velocity for the cylinder in mm per second
    /// - `set_dist` is the set distance for the cylinder in mm
    fn measure(&mut self, delta : Delta, omega : Omega, set_gamma : Gamma, accuracy : u64) -> bool {
        self.cylinder.measure(
            delta, omega, self.gamma_for_super(set_gamma), accuracy)
    }

    fn measure_async(&mut self, delta : Delta, omega : Omega, accuracy : u64) {
        self.cylinder.measure_async(
            delta, omega, accuracy)
    }
    
    // Forces
        fn apply_load_force(&mut self, force : Force) {
            self.cylinder.apply_load_force(force)
        }

        fn apply_load_inertia(&mut self, inertia : Inertia) {
            self.cylinder.apply_load_inertia(inertia)
        }
    // 
}