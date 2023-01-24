use crate::math::actors;

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
    pub relev : Vec<[f32; N]>,
    pub omegas : Vec<[f32; N]>,

    pub scalars : Vec<f32>,
    pub times : Vec<f32>
}

impl<const N : usize> CompPath<N> 
{
    pub fn new(phis : Vec<[f32; N]>, relev : Vec<[f32; N]>) -> Self {
        CompPath {
            phis,
            relev,
            omegas: vec![],
            times: vec![], 
            scalars: vec![]
        }
    }

    pub fn add_node(&mut self, omegas : [f32; N], scalar : f32, time : f32) {
        self.omegas.push(omegas);
        self.times.push(time);
        self.scalars.push(scalar);
    }

    pub fn generate(&mut self, comps : &[Box<dyn Component>; N], vel_0 : [f32; N], vel_max : f32) {
        for i in 0 .. (self.phis.len() - 1) {
            let compl = actors::compl_times(comps, self.phis[i], self.phis[i + 1], vel_0, self.relev[i], vel_max);
            let ( f_s, index_min, _ ) = actors::f_s(&compl);

            if f_s < 1.0 {
                // Backwards correction
            }

            let dt = compl[index_min][0][0];
            let omega_fixed = compl[index_min][1][0];

            let mut omegas = [0.0; N];

            for n in 0 .. N {
                let factor = self.relev[n][i] / self.relev[n][index_min];
                omegas[n] = omega_fixed * factor;
            }

            self.add_node(omegas, f_s, dt);
        }
    }
}