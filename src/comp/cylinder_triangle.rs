use serde::{Serialize, Deserialize};

use crate::{SyncComp, Setup};
use crate::comp::Cylinder;
use crate::data::LinkedData;
use crate::meas::SimpleMeas;
use crate::units::*;

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

// impl crate::math::MathActor for CylinderTriangle {
//     fn accel_dyn(&self, omega : Omega, gamma : Gamma) -> Alpha {
//         self.alpha_for_this(self.cylinder.accel_dyn(
//             self.omega_for_super(omega, gamma), self.gamma_for_super(gamma)), self.gamma_for_super(gamma))
//     }
// }

impl Setup for CylinderTriangle {
    fn setup(&mut self) -> Result<(), crate::Error> { 
        self.cylinder.setup()?;
        self.cylinder.write_gamma(Gamma(self.l_a.max(self.l_b)));

        Ok(())
    }
}

impl SimpleMeas for CylinderTriangle {
    fn init_meas(&mut self, pin_meas : u8) {
        self.cylinder.init_meas(pin_meas)
    }
}

impl SyncComp for CylinderTriangle {
    // Data
        fn consts<'a>(&'a self) -> &'a crate::StepperConst {
            self.cylinder.consts()    
        }

        fn link<'a>(&'a self) -> &'a crate::data::LinkedData {
            self.cylinder.link()
        }

        fn vars<'a>(&'a self) -> &'a crate::data::CompVars {
            self.cylinder.vars()
        }
    // 

    // Super 
        fn super_comp(&self) -> Option<&dyn SyncComp> {
            Some(&self.cylinder)
        }

        fn super_comp_mut(&mut self) -> Option<&mut dyn SyncComp> {
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
        fn write_link(&mut self, lk : LinkedData) {
            self.cylinder.write_link(lk);
        }
    //

    // JSON I/O
        fn to_json(&self) -> Result<serde_json::Value, serde_json::Error> {
            serde_json::to_value(self)
        }
    //

    /// See [SyncComp::drive_rel()]
    /// - `dist`is the angular distance to be moved (Unit radians)
    /// - `vel` is the cylinders extend velocity (Unit mm per second)
    fn drive_rel(&mut self, mut delta : Delta, omega : Omega) -> Result<Delta, crate::Error> {
        let gamma = self.gamma();
        
        delta = self.delta_for_super(delta, gamma);
        delta = self.cylinder.drive_rel(delta, omega)?;

        Ok(self.delta_for_this(delta, self.gamma_for_super(gamma)))
    }

    /// See [SyncComp::drive_abs]
    /// - `dist`is the angular distance to be moved (Unit radians)
    /// - `vel` is the cylinders extend velocity (Unit mm per second)
    fn drive_abs(&mut self, mut gamma : Gamma, omega : Omega) -> Result<Delta, crate::Error> {
        gamma = self.gamma_for_super(gamma);

        let delta = self.cylinder.drive_abs(gamma, omega)?;

        Ok(self.delta_for_this(delta, gamma))
    }

    /// See [SyncComp::measure()]
    /// - `dist` is the maximum distance for the cylinder in mm
    /// - `vel` is the maximum linear velocity for the cylinder in mm per second
    /// - `set_dist` is the set distance for the cylinder in mm
    fn measure(&mut self, delta : Delta, omega : Omega, set_gamma : Gamma) -> Result<Delta, crate::Error> {
        self.cylinder.measure(
            delta, omega, self.gamma_for_super(set_gamma))
    }
    
    // Forces
        fn apply_force(&mut self, force : Force) {
            self.cylinder.apply_force(force)
        }

        fn apply_inertia(&mut self, inertia : Inertia) {
            self.cylinder.apply_inertia(inertia)
        }
    // 
}