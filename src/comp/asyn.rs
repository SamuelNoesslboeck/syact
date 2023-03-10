// use crate::data::CompVars;

pub enum Direction {
    CW, 
    CCW, 
    None
}

pub trait AsyncComp {
    fn drive(&mut self, dir : Direction, speed_perc : f32) -> Result<(), crate::Error>; 

    // Data
        fn dir(&self) -> Direction;
        
        fn speed_perc(&self) -> f32;

        // fn vars<'a>(&'a self) -> &'a CompVars;
    //
}