use crate::Component;
use crate::units::*;

pub trait AsyncComp : Component {
    /// Moves the component by the relative distance as fast as possible. \
    /// To wait unti the movement operation is completed, use the [await_inactive](Component::await_inactive()) function
    fn drive_rel_async(&mut self, delta : Delta, omega : Omega);

    /// Moves the component to the given position as fast as possible. \
    /// To wait unti the movement operation is completed, use the [await_inactive](Component::await_inactive()) function
    fn drive_abs_async(&mut self, gamma : Gamma, omega : Omega);

    /// Measure the component by driving the component with the velocity `omega` until either the measurement condition is true or the maximum distance `delta` 
    /// is reached. The lower the `accuracy`, the higher are the computational difficulties, as the function checks more often if the measure pin has a HIGH signal
    fn measure_async(&mut self, delta : Delta, omega : Omega, set_gamma : Gamma, accuracy : u64);

    /// Halts the thread until the movement of the component has finished. \
    /// Do only use it after an async movement has been triggered before!
    fn await_inactive(&self);
}