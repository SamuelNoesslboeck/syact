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

