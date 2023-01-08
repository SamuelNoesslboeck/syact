// Renamed types
/// Curve to drive for the stepper motor, the values represent waiting times between steps
pub type StepperCurve = Vec<f32>;

/// A renamed type representing a curve, with the first vector of points being the differences in time between each point dt 
type CurvePoints = (Vec<f32>, Vec<f32>);

pub type PathPhi = CurvePoints;
pub type PathOmegas = CurvePoints;
pub type PathAlphas = CurvePoints;

pub struct StepperPath( 
    /// **dt**, time `dt` passed since last node
    Vec<f32>, 
    /// **phi**, angle `phi` at the current time in radians
    Vec<f32>,
    /// **omega**, angluar velocity `omega` at the current time in radians per second
    Vec<f32>, 
    /// **alpha**, angluar acceleration `alpha` at the current time in radians per second squared
    Vec<f32>
);

pub fn path_derivative(path : &CurvePoints, startval : &f32) -> Vec<f32> {
    let (dts, vals) = path;
    let mut deri : Vec<f32> = Vec::new();

    for i in 0 .. vals.len() {
        let val = vals[i];
        deri.push((val - vals.get(i - 1).unwrap_or(startval)) / dts[i])
    }
    
    deri
}

pub fn create_node_path(phiCurve : PathPhi) -> StepperPath {
    let ( dts, phis ) = phiCurve;
    let omegas = path_derivative(&( dts, phis ), phis.first().unwrap());
    let alphas = path_derivative(&( dts, omegas ), &0.0);

    StepperPath(
        dts, 
        phis,
        omegas,
        alphas
    )
} 

pub fn path_correction(path : &mut Vec<StepperPath>) {
    
}