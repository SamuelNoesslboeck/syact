use crate::{StepperData, ctrl::{StepperCtrl, LimitType, LimitDest}, UpdateFunc, Vec3};

/// Cylinder component struct
pub struct Cylinder
{
    /// Data of the connected stepper motor
    pub ctrl : Box<dyn StepperCtrl>,

    /// Distance traveled per rad [in mm]   \
    /// f_rte = pitch / (2pi)
    pub rte_ratio : f32,
}

impl Cylinder
{
    /// Create a new cylinder instance
    pub fn new(ctrl : Box<dyn StepperCtrl>, rte_ratio : f32) -> Self {
        return Cylinder {
            ctrl,
            rte_ratio
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

    // Limits
        pub fn conv_limit(&self, limit : LimitType) -> LimitType {
            match limit {
                LimitType::Distance(dist) => LimitType::Steps(self.ctrl.ang_to_steps_dir(self.phi_c(dist))), 
                LimitType::Steps(_) => limit,
                _ => LimitType::None
            }
        }

        pub fn set_limit(&mut self, limit_max : LimitType, limit_min : LimitType) {
            self.ctrl.set_limit(self.conv_limit(limit_min), self.conv_limit(limit_max));
        }

        pub fn get_limit_dest(&self, dist : f32) -> LimitDest {
            match self.ctrl.get_limit_dest(self.ctrl.ang_to_steps_dir(self.phi_c(dist))) {
                LimitDest::Minimum(ang) => LimitDest::Minimum(self.dis_c(ang)), 
                LimitDest::Maximum(ang) => LimitDest::Maximum(self.dis_c(ang)), 
                other => other
            }
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
            self.ctrl.apply_load_j(mass * (self.rte_ratio / 1000.0).powi(2));
        }

        pub fn apply_load_f(&mut self, force : f32) {
            self.ctrl.apply_load_t(force * self.rte_ratio / 1000.0);
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
    pub fn len_for_gam(&self, gam : f32) -> f32 {
        (self.l_a.powi(2) + self.l_b.powi(2) - 2.0 * self.l_a * self.l_b * gam.cos()).powf(0.5)
    }

    /// Returns the angle gamma for the given cylinder length _(len < (l_a + l_b))_
    pub fn gam_for_len(&self, len : f32) -> f32 {
        ((self.l_a.powi(2) + self.l_b.powi(2) - len.powi(2)) / 2.0 / self.l_a / self.l_b).acos()
    }

    // Angle
        pub fn get_gam(&self) -> f32 {
            self.gam_for_len(self.cylinder.length())
        }

        pub fn set_gam(&mut self, gam : f32, v_max : f32) {
            self.cylinder.extend(self.len_for_gam(gam) - self.cylinder.length(), v_max);
        }
    //

    pub fn write_gam(&mut self, gam : f32) {
        self.cylinder.write_length(self.len_for_gam(gam))
    }

    pub fn measure(&mut self, max_dis : f32, v_max : f32, dir : bool, set_angle : f32, accuracy : u64) {
        self.cylinder.measure(max_dis, v_max, dir, self.len_for_gam(set_angle), accuracy);
    }
    
    // Limit
        pub fn set_limit(&mut self, limit_min : LimitType, limit_max : LimitType) {
            self.cylinder.set_limit(limit_max, limit_min)
        }

        pub fn get_limit_dest(&self, gam : f32) -> LimitDest {
            match self.cylinder.get_limit_dest(self.len_for_gam(gam)) {
                LimitDest::Maximum(dist) => LimitDest::Maximum(self.gam_for_len(dist)),
                LimitDest::Minimum(dist) => LimitDest::Minimum(self.gam_for_len(dist)),
                other => other  
            }
        }
    //
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

        /// Returns the angle for the motor from a given bearing angle
        pub fn ang_for_bear(&self, ang : f32) -> f32 {
            ang * self.ratio
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

    // Limits
        pub fn set_limit(&mut self, limit_min : LimitType, limit_max : LimitType) {
            self.ctrl.set_limit(
                match limit_min {
                    LimitType::Angle(ang) => LimitType::Steps(self.ctrl.ang_to_steps_dir(self.ang_for_motor(ang))), 
                    _ => LimitType::None
                }, match limit_max {
                    LimitType::Angle(ang) => LimitType::Steps(self.ctrl.ang_to_steps_dir(self.ang_for_motor(ang))),
                    _ => LimitType::None
                }
            )
        }

        pub fn get_limit_dest(&self, gam : f32) -> LimitDest {
            match self.ctrl.get_limit_dest(self.ctrl.ang_to_steps_dir(self.ang_for_motor(gam))) {
                LimitDest::Maximum(dist) => LimitDest::Maximum(self.ang_for_bear(dist)),
                LimitDest::Minimum(dist) => LimitDest::Minimum(self.ang_for_bear(dist)),
                other => other  
            }
        }
    //

    pub fn apply_load_j(&mut self, inertia : f32) {
        self.ctrl.apply_load_j(inertia * self.ratio);
    }

    pub fn apply_load_t(&mut self, torque : f32) {
        self.ctrl.apply_load_t(torque * self.ratio);
    }
}

// Tools
pub trait Tool
{
    /// Returns the characteristic vector of the tool
    fn get_vec(&self) -> Vec3;

    /// Returns the tool inhertia 
    fn get_inertia(&self) -> f32;

    /// Return the tool mass
    fn get_mass(&self) -> f32;
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

    fn get_inertia(&self) -> f32 {
        return 0.0;
    }

    fn get_mass(&self) -> f32 {
        return 0.0;
    }
}