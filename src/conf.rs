use std::sync::Arc;

use glam::{Mat3, Vec3};

use crate::{LinkedData, Omega, Gamma, Delta, Phi};
use crate::comp::Tool;

// Submodules
mod elem;
pub use elem::*;

mod json;
pub use json::*;


#[derive(Debug)]
pub struct MachineConfig<const N : usize, const D : usize, const A : usize>
{
    pub name: String,

    pub lk : Arc<LinkedData>,

    pub anchor : Vec3,
    pub dims : [Vec3; D],
    pub axes : [Vec3; A],

    pub tools : Vec<Box<dyn Tool + Send>>,

    pub vels : [Omega; N],
    pub home : [Gamma; N],
    pub meas_dist : [Delta; N],
    
    pub ang : [AngleData; N],
    pub sim : [SimData; N],
    pub meas : [MeasInstance; N],
    pub limit : [LimitDecl; N]
}

impl<const N : usize, const D : usize, const A : usize> Default for MachineConfig<N, D, A> 
{
    fn default() -> Self {
        Self {
            name: String::new(),
            lk: Default::default(),

            anchor: Default::default(),
            dims: [Default::default(); D],
            axes: [Default::default(); A], 

            tools: vec![],

            vels: [Omega::ZERO; N],
            home: [Gamma::ZERO; N],
            meas_dist: [Delta::ZERO; N],
            
            ang: [Default::default(); N],
            sim: [Default::default(); N],
            meas: [Default::default(); N],
            limit: [Default::default(); N]
        }
    }
}

impl<const N : usize, const D : usize, const A : usize> MachineConfig<N, D, A>
{
    pub fn get_axes(&self, angles : &[Phi; A]) -> Vec<Mat3> {
        let mut matr = vec![];

        for i in 0 .. A {
            let axis_vec = Vec3::from(self.axes[i]).normalize();

            matr.push(
                if axis_vec == Vec3::X {
                    Mat3::from_rotation_x(angles[i].0)
                } else if axis_vec == Vec3::Y {
                    Mat3::from_rotation_y(angles[i].0)
                } else if axis_vec == Vec3::Z {
                    Mat3::from_rotation_z(angles[i].0)
                } else {
                    Mat3::ZERO
                }
            );
        }

        matr
    }
}

impl<const N : usize, const D : usize, const A : usize> MachineConfig<N, D, A>
{
    pub fn gammas_from_phis(&self, phis : [Phi; N]) -> [Gamma; N] {
        let mut gammas = [Gamma::ZERO; N];

        for i in 0 .. N {
            gammas[i] = if self.ang[i].counter { 
                -phis[i].force_to_gamma() 
            } else { 
                phis[i].force_to_gamma()
            } - self.ang[i].offset;
        }

        gammas
    }

    pub fn phis_from_gammas(&self, gammas : [Gamma; N]) -> [Phi; N] {
        let mut phis = [Phi::ZERO; N];

        for i in 0 .. N {
            phis[i] = (if self.ang[i].counter { 
                -gammas[i]
            } else { 
                gammas[i]
            } - self.ang[i].offset).force_to_phi();
        }

        phis
    }
}