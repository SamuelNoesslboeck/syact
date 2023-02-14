use std::sync::Arc;

use glam::{Mat3, Vec3};

use crate::LinkedData;
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

    pub vels : [f32; N],
    pub home : [f32; N],
    pub meas_dist : [f32; N],
    
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

            vels: [0.0; N],
            home: [0.0; N],
            meas_dist: [0.0; N],
            
            ang: [Default::default(); N],
            sim: [Default::default(); N],
            meas: [Default::default(); N],
            limit: [Default::default(); N]
        }
    }
}

impl<const N : usize, const D : usize, const A : usize> MachineConfig<N, D, A>
{
    pub fn get_axes(&self, angles : &[f32; A]) -> Vec<Mat3> {
        let mut matr = vec![];

        for i in 0 .. A {
            let axis_vec = Vec3::from(self.axes[i]).normalize();

            matr.push(
                if axis_vec == Vec3::X {
                    Mat3::from_rotation_x(angles[i])
                } else if axis_vec == Vec3::Y {
                    Mat3::from_rotation_y(angles[i])
                } else if axis_vec == Vec3::Z {
                    Mat3::from_rotation_z(angles[i])
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
    pub fn convert_angles(&self, angles : [f32; N], to_comp : bool) -> [f32; N] {
        let mut full_ang = [0.0; N];

        for i in 0 .. N {
            full_ang[i] = if self.ang[i].counter { 
                -angles[i] 
            } else { 
                angles[i] 
            } + if to_comp { 
                -self.ang[i].offset 
            } else {
                self.ang[i].offset
            };
        }

        full_ang
    }
}