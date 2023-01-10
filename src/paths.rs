use std::vec;

use crate::Component;

// Renamed types
/// Curve to drive for the stepper motor, the values represent waiting times between steps
pub type StepperCurve = Vec<f32>;

/// A renamed type representing a curve, with the first vector of points being the differences in time between each point dt 
type CurvePoints = (Vec<f32>, Vec<f32>);

pub type PathPhi = CurvePoints;
pub type PathOmegas = CurvePoints;
pub type PathAlphas = CurvePoints;

pub struct StepperPath 
{
    /// **dt**, time `dt` passed since last node
    pub dts : Vec<f32>, 
    /// **phi**, angle `phi` at the current time in radians
    pub phis : Vec<f32>,
    /// **omega**, angluar velocity `omega` at the current time in radians per second
    pub omegas : Vec<f32>, 
    /// **alpha**, angluar acceleration `alpha` at the current time in radians per second squared
    pub alphas : Vec<f32>
}

// PathPhi functions
    pub fn pathphi_new() -> PathPhi {
        ( Vec::new(), Vec::new() )
    }

    pub fn pathphi_push(path : &mut PathPhi, values : (f32, f32)) {
        path.0.push(values.0);
        path.1.push(values.1);
    }
//

pub fn path_correction(phi_curves : &mut Vec<PathPhi>, comps : &mut Vec<&mut dyn Component>, node_count : usize) -> Vec<StepperPath> {
    let mut new_dts : Vec<f32> = vec![];

    let mut omegas : Vec<Vec<f32>> = vec![vec![ 0.0 ]; phi_curves.len()];
    let mut alphas : Vec<Vec<f32>> = vec![vec![]; phi_curves.len()];

    for n in 0 .. node_count {
        let mut fs_min = 1.0; 
        let mut dt_new = phi_curves[0].0[n];

        for i in 0 .. phi_curves.len() {
            let comp = &comps[i];

            let dts = &phi_curves[i].0;
            let phis = &phi_curves[i].1;

            // Calculate values 
            let delta_phi = phis.get(n + 1).unwrap_or(phis.last().unwrap()) - phis[n];
            let alpha = 2.0 * (delta_phi - omegas[i][n]*dts[n]) / dts[i].powi(2);

            alphas[i].push(alpha);

            if i != (dts.len() - 1) {
                let new_omega = omegas[i][n] + alphas[i][n]*dts[n];
                omegas[i].push(new_omega);
            }

            let f_s = comp.accel_dyn(omegas[i][n], phi_curves[i].1[n]) / alphas[i][n];
            
            if f_s < 0.0 {
                continue;
            }

            if f_s < fs_min {
                fs_min = f_s;

                // Modfiy curve
                let a = -f_s * alphas[n][i] / 2.0;  
                let b = -omegas[n].get(n - 1).unwrap_or(&0.0);
                let c = phis[n] - phis.get(n - 1).unwrap_or(&phis[n]);
        
                let p = b / a / 2.0; 
                let q = c / a; 
        
                dt_new = -p + (p.powi(2) - q).powf(0.5);
            }
        }

        new_dts.push(dt_new);

        if fs_min < 1.0 {
            for i in 0 .. phi_curves.len() {
                let dts = &phi_curves[i].0;
                let phis = &phi_curves[i].1;

                // Calculate values 
                let delta_phi = phis.get(n + 1).unwrap_or(phis.last().unwrap()) - phis[n];
                let alpha = 2.0 * (delta_phi - omegas[i][n]*dts[n]) / dts[i].powi(2);

                alphas[i][n] = alpha;

                if i != (dts.len() - 1) {
                    let new_omega = omegas[i][n] + alphas[i][n]*dts[n];
                    omegas[i][n] = new_omega;
                }
            }
        }
    }

    let mut paths = vec![];

    for i in 0 .. phi_curves.len() {
        paths.push(StepperPath {
            dts: new_dts.clone(),
            phis: phi_curves[i].1.clone(),
            omegas: omegas[i].clone(),
            alphas: alphas[i].clone()
        })
    }

    paths
}