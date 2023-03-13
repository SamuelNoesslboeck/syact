/// A structure for taking basic measurements
pub trait SimpleMeas 
{
    /// Initialize everything required for measurements
    fn init_meas(&mut self, pin_meas : u8);
}