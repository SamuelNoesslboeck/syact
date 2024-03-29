use serde::{Serialize, Deserialize};

use crate::{SyncComp, Setup, Stepper};
use crate::comp::Cylinder;
use crate::comp::stepper::StepperComp;
use crate::data::CompData;
use crate::units::*;

/// A cylinder triangle using a cylinder with a stepper motor to power itself
pub type StepperCylTriangle = CylinderTriangle<Stepper>;

/// A component representing a cylinder connected to two segments with constant lengths, forming a triangular shape
/// 
/// # Super Component
/// 
/// Uses a [Cylinder] as parent component and as the triangular shape makes a constant angular velocity
/// very calculation expensive, all maximum velocites are referencing the cylinder
/// 
/// # Angles and lengths
/// 
/// The struct uses the default labeling of a mathematical triangles with the sides a, b and c. Here, a and b are the 
/// constant lengths and c being the variable cylinder length. All angles used are alpha, beta and gamma, all depending 
/// on the variable length c, making them also variable. The most relevant angle being gamma, as it is the opposing angle to 
/// the length c, representing the 
#[derive(Debug, Serialize, Deserialize)]
pub struct CylinderTriangle<C : SyncComp> {
    /// The cylinder of the triangle, being the *parent component* for this one
    pub cylinder : Cylinder<C>,

    // Triangle
    /// The constant length of the first triangle component in millimeters
    pub l_a : f32,
    /// The constant length of the second triangle component in millimeters
    pub l_b : f32,
}

impl<C : SyncComp> CylinderTriangle<C> {
    /// Creates a new instance of a [CylinderTriangle], 
    /// writing an initial length of the longer segments the cylinder, preventing initial calculation errors
    pub fn new(cylinder : Cylinder<C>, l_a : f32, l_b : f32) -> Self {
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
            (self.l_a / self.gamma_for_parent(gam).0 * gam.sin()).asin()
        }

        /// Returns the beta angle (opposing to the b-segment) for a given gamma angle `gam`
        pub fn beta_for_gam(&self, gam : Gamma) -> f32 {
            (self.l_b / self.gamma_for_parent(gam).0 * gam.sin()).asin()
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
//             self.omega_for_parent(omega, gamma), self.gamma_for_parent(gamma)), self.gamma_for_parent(gamma))
//     }
// }

impl<C : SyncComp> Setup for CylinderTriangle<C> {
    fn setup(&mut self) -> Result<(), crate::Error> { 
        self.cylinder.setup()?;
        self.cylinder.write_gamma(Gamma(self.l_a.max(self.l_b)));

        Ok(())
    }
}

impl<C : SyncComp> SyncComp for CylinderTriangle<C> {
    // Data
        fn data<'a>(&'a self) -> &'a crate::data::CompData {
            self.cylinder.data()
        }

        fn vars<'a>(&'a self) -> &'a crate::data::CompVars {
            self.cylinder.vars()
        }
    // 

    // Super 
        fn parent_comp(&self) -> Option<&dyn SyncComp> {
            Some(&self.cylinder)
        }

        fn parent_comp_mut(&mut self) -> Option<&mut dyn SyncComp> {
            Some(&mut self.cylinder)
        }

        /// Returns the cylinder length for the given angle gamma
        fn gamma_for_parent(&self, gam : Gamma) -> Gamma {
            Gamma((self.l_a.powi(2) + self.l_b.powi(2) - 2.0 * self.l_a * self.l_b * gam.0.cos()).powf(0.5))
        }

        /// Returns the angle gamma for the given cylinder length _(len < (l_a + l_b))_
        fn gamma_for_this(&self, len : Gamma) -> Gamma {
            Gamma(((self.l_a.powi(2) + self.l_b.powi(2) - len.0.powi(2)) / 2.0 / self.l_a / self.l_b).acos())
        }

        fn omega_for_this(&self, parent_omega : Omega, this_gamma : Gamma) -> Omega {
            parent_omega / self.l_a * self.beta_for_gam(this_gamma).sin()
        }
    // 

    // Link
        fn write_data(&mut self, data : CompData) {
            self.cylinder.write_data(data);
        }
    //

    // Omega max 
        fn omega_max(&self) -> Omega {
            self.cylinder.omega_max()
        }

        fn set_omega_max(&mut self, omega_max : Omega) {
            self.cylinder.set_omega_max(omega_max)
        }
    // 

    /// See [SyncComp::drive_rel()]
    /// - `dist`is the angular distance to be moved (Unit radians)
    /// - `vel` is the cylinders extend velocity (Unit mm per second)
    fn drive_rel(&mut self, mut delta : Delta, speed_f : f32) -> Result<Delta, crate::Error> {
        let gamma = self.gamma();
        
        delta = self.delta_for_parent(delta, gamma);
        delta = self.cylinder.drive_rel(delta, speed_f)?;

        Ok(self.delta_for_this(delta, self.gamma_for_parent(gamma)))
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

impl<C : StepperComp> StepperComp for CylinderTriangle<C> {
    #[inline]
    fn parent_stepper_comp(&self) -> Option<&dyn StepperComp> {
        Some(&self.cylinder)
    }

    #[inline]
    fn parent_stepper_comp_mut(&mut self) -> Option<&mut dyn StepperComp> {
        Some(&mut self.cylinder)
    }

    #[inline]
    fn motor(&self) -> &dyn crate::prelude::StepperMotor {
        self.cylinder.motor()
    }

    #[inline]
    fn motor_mut(&mut self) -> &mut dyn crate::prelude::StepperMotor {
        self.cylinder.motor_mut()
    }
}