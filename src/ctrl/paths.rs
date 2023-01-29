use crate::math::actors;

use super::*;

// Helper
    fn scale_array<const N : usize>(mut array : [f32; N], scalar : f32) -> [f32; N] {
        for i in 0 .. N {
            array[i] *= scalar;
        }

        array
    }
//

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

    // pub scalars : Vec<f32>,
    pub times : Vec<f32>,
    pub accels : Vec<[f32; N]>
}

impl<const N : usize> CompPath<N> 
{
    pub fn new(phis : Vec<[f32; N]>, relev : Vec<[f32; N]>) -> Self {
        CompPath {
            phis,
            relev,
            omegas: vec![],
            times: vec![],
            accels : vec![]
            // scalars: vec![]
        }
    }

    pub fn fill_empty(&mut self, length : usize) {
        for _ in 0 .. length {
            self.omegas.push([f32::INFINITY; N]);
            self.times.push(0.0);
            self.accels.push([f32::INFINITY; N]);
        }
    }

    pub fn add_node(&mut self, omegas : [f32; N], scalar : f32, time : f32) {
        self.omegas.push(omegas);
        self.times.push(time);
        // self.scalars.push(scalar);
    }

    pub fn edit_node(&mut self, index : usize, omegas : [f32; N], scalar : f32, time : f32) {
        self.omegas[index] = omegas; 
        // self.scalars[index] = scalar;
        self.times[index] = time;
    }

    pub fn corr_node(&mut self, comps : &[Box<dyn Component>; N], compl : &[[[f32; 2]; 3]; N], index : usize, o_index : usize) -> bool {
        let ( f_s, index_min, _ ) = actors::f_s(&compl);

        let dt = compl[index_min][0][0];
        let omega_fixed = compl[index_min][1][0];

        let mut omegas = [0.0; N];
        let mut accels = [0.0; N];

        // For each component
        for n in 0 .. N {
            let factor = self.relev[index][n] / self.relev[index][index_min];
            omegas[n] = omega_fixed * factor;
            accels[n] = compl[n][2][0];
        }

        if f_s < 1.0 {
            self.backwards_corr(comps, scale_array(omegas, f_s), o_index);

            true
        } else {
            if omega_fixed.abs() < self.omegas[o_index][index_min].abs() {
                self.omegas[o_index] = omegas;
                self.times[index] = dt;
                self.accels[index] = accels;   
    
                true
            } else {
                false
            }
        }
    }

    pub fn backwards_corr(&mut self, comps : &[Box<dyn Component>; N], omegas : [f32; N], mut index : usize) {
        self.omegas[index] = omegas;

        loop {
            if index == 0 {
                break; 
            }

            let compl = actors::compl_times(comps, self.phis[index - 1], self.phis[index], 
                self.omegas[index], self.relev[index - 1]
            );

            if !self.corr_node(comps, &compl, index, index - 1) {
                break;
            }

            index -= 1;
        }
    }

    pub fn generate(&mut self, comps : &[Box<dyn Component>; N], vel_0 : [f32; N], vel_end : [f32; N]) {
        let path_len = self.phis.len(); 

        self.fill_empty(path_len);
        self.omegas[0] = vel_0;

        for i in 0 .. (path_len - 1) {
            let compl = actors::compl_times(comps, self.phis[i], self.phis[i + 1], 
            self.omegas[i], self.relev[i]
            );
            
            self.corr_node(comps, &compl, i, i + 1);
        }
        
        self.backwards_corr(comps, vel_end, path_len - 1);

        // if f_s < 1.0 {
        //     let compl_corr = actors::compl_times(comps, self.phis[i - 1], self.phis[i], 
        //         self.omegas[i], self.relev[i]
        //     );

        //     let ( f_s, index_min, _ ) = actors::f_s(&compl);

        //     let dt = compl[index_min][0][0];
        //     let omega_fixed = compl[index_min][1][0];

        //     let mut omegas = [0.0; N];
        //     let mut accels = [0.0; N];


        // }

        // self.omegas[path_len - 1] = vel_end;

        // for i in 2 .. (path_len - 1) {
        //     let index = path_len - i;

        //     let compl = actors::compl_times(comps, self.phis[index], self.phis[index + 1], 
        //         self.omegas[index + 1], self.relev[index]
        //     );
        //     let ( _, index_min, _ ) = actors::f_s(&compl);

        //     let dt = compl[index_min][0][0];
        //     let omega_fixed = compl[index_min][1][0];

        //     let mut omegas = [0.0; N];
        //     let mut accels = [0.0; N];

        //     // For each component
        //     for n in 0 .. N {
        //         let factor = self.relev[index][n] / self.relev[index][index_min];
        //         omegas[n] = omega_fixed * factor;
        //         accels[n] = compl[n][2][0];
        //     }

        //     if dt > self.times[index] {
        //         self.omegas[index] = omegas;
        //         self.times[index] = dt;
        //         self.accels[index] = accels;
        //     }
        // }
    }

    pub fn debug_path(&self, index : usize) {
        println!("index\t|d-phi\t|omega\t|relev\t|times\t|accel");
        for i in 0 .. self.phis.len() {
            println!("{}\t|{}\t|{}\t|{}\t|{}\t|{}\t", 
                i, self.phis.get(i + 1).unwrap_or(&self.phis[i])[index] - self.phis[i][index], self.omegas[i][index], self.relev[i][index], self.times[i], self.accels[i][index]);
        }
    }
}