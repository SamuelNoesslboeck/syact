use crate::math::{self, actors};

use super::*;

pub struct StepperCurve 
{
    pub curve : Vec<[f32; 2]>
}

impl StepperCurve
{
    pub fn curve_at(&self, step : f32) -> Option<f32> {
        if step > (self.curve.len() as f32 - 1.0) {
            return None;
        }

        if step < 0.0 {
            return None;
        }

        let index = step.floor();
        Some(self.curve[index as usize][1] + self.curve[index as usize][0]*(step - index))
    }

    pub fn translate(&self, new_length : usize) -> Self {
        let mut new_curve = vec![ [0.0, 0.0] ]; 
        let f_s = (self.curve.len() as f32) / (new_length as f32);

        let mut current;
        let mut last = 0.0;
        for i in 0 .. new_length {
            current = self.curve_at(f_s * (i as f32 + 1.0)).unwrap();   
            new_curve.push([current - last, current]);
            last = current;
        }

        StepperCurve { 
            curve: new_curve 
        }
    }
}

pub struct CompPath<const N : usize>
{
    pub phis : Vec<[f32; N]>,
    pub omegas : Vec<[f32; N]>,
    pub times : Vec<[f32; N]>,
    pub scalars : Vec<[f32; N]>
}

impl<const N : usize> CompPath<N> 
{
    pub fn new(phis : Vec<[f32; N]>) -> Self {
        CompPath {
            phis,
            omegas: vec![],
            times: vec![], 
            scalars: vec![]
        }
    }

    pub fn add_node(&mut self, omegas : [f32; N], times : [f32; N], scalars : [f32; N]) {
        self.omegas.push(omegas);
        self.times.push(times);
        self.scalars.push(scalars);
    }

    pub fn generate(&mut self, comps : &[Box<dyn Component>; N], vel_0 : [f32; N], vel_max : f32) {
        for i in 0 .. (self.phis.len() - 1) {
            let relv = actors::relv_factors(self.phis[i], self.phis[i + 1]);
            let compl = actors::compl_times(comps, self.phis[i], self.phis[i + 1], vel_0, vel_max);
            let ( f_s, index_min, index_max ) = actors::f_s(&compl);

            if f_s < 1.0 {
                // Backwards correction
            }


        }
    }
}