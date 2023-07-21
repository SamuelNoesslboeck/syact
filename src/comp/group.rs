use crate::{Setup, SyncComp};
use crate::units::*;

/// A group of synchronous components  
/// This trait allows a lot of functions to be used for all components at once.
pub trait SyncCompGroup<T, const C : usize> : Setup 
where T : SyncComp + ?Sized + 'static
{
    // Iteration
        /// Execute a given function `func` for every element given by a readonly reference and it's index in the component group.
        /// 
        /// # Returns
        /// 
        /// Returns a fixed-sized array of the result type of the closure. The closure may not fail, see `try_for_each()` if a `Result` is required
        fn for_each<'a, F, R>(&'a self, func : F) -> [R; C]
        where 
            F : FnMut(&'a T, usize) -> R,
            R : Copy + Default;

        /// Execute a given function `func` for every element given by a mutable reference and it's index in the component group 
        /// 
        /// # Returns
        /// 
        /// Returns a fixed-sized array of the result type of the closure. The closure may not fail, see `try_for_each_mut()` if a `Result` is required
        fn for_each_mut<F, R>(&mut self, func : F) -> [R; C]
        where 
            F : FnMut(&mut T, usize) -> R,
            R : Copy + Default;

        /// Execute a given function `func` for every element given by a mutable reference and it's index in the component group 
        /// 
        /// # Returns
        /// 
        /// Returns a fixed-sized array of the result type of the closure, wrapped into a `Result`, the closure has to return a `Result`
        /// 
        /// # Error
        /// 
        /// Stops executing if one of the components throws an error
        fn try_for_each<'a, F, R, E>(&'a self, func : F) -> Result<[R; C], E>
        where 
            F : FnMut(&'a T, usize) -> Result<R, E>,
            R : Copy + Default;

        /// Execute a given function `func` for every element given by a mutable reference and it's index in the component group 
        /// 
        /// # Returns
        /// 
        /// Returns a fixed-sized array of the result type of the closure, wrapped into a `Result`, the closure has to return a `Result`
        /// 
        /// # Error
        /// 
        /// Stops executing if one of the components throws an error
        fn try_for_each_mut<F, R, E>(&mut self, func : F) -> Result<[R; C], E>
        where 
            F : FnMut(&mut T, usize) -> Result<R, E>,
            R : Copy + Default;

        /// Execute a given function `func` for every element given by a readonly reference and it's index in the component group.
        /// 
        /// # Returns
        /// 
        /// Returns a dynamically sized array of the return time. The advantage is that the value does not have to implement `Copy` nor `Default`.
        /// The closure may not fail, see `try_for_each()` if a `Result` is required.
        fn for_each_dyn<'a, F, R>(&'a self, func : F) -> Vec<R>
        where 
            F : FnMut(&'a T, usize) -> R;
    //

    // Data
        /// Runs [SyncComp::write_data()] for all components in the group. Note that the function is using the same 
        /// [CompData](crate::data::CompData) for all components
        fn write_data(&mut self, data : crate::data::CompData) {
            self.for_each_mut(move |comp, _| {
                comp.write_data(data.clone());
            });
        } 
    //

    /// Runs [SyncComp::drive_rel()] for all components
    fn drive_rel(&mut self, deltas : [Delta; C], speed_f : [f32; C]) -> Result<[Delta; C], crate::Error> {
        self.try_for_each_mut(|comp, index| {
            comp.drive_rel(deltas[index], speed_f[index])  
        })
    }

    /// Runs [SyncComp::drive_abs()] for all components
    fn drive_abs(&mut self, gamma : [Gamma; C], speed_f : [f32; C]) -> Result<[Delta; C], crate::Error>  {
        self.try_for_each_mut(|comp, index| {
            comp.drive_abs(gamma[index], speed_f[index])
        })
    }

    // Async
        /// Runs [SyncComp::drive_rel_async()] for all components
        /// 
        /// # Features
        /// 
        /// Only available if the "std" feature is enabled
        #[cfg(feature = "embed-thread")]
        fn drive_rel_async(&mut self, deltas : [Delta; C], speed_f : [f32; C]) -> Result<(), crate::Error> {
            self.try_for_each_mut(|comp, index| {
                comp.drive_rel_async(deltas[index], speed_f[index])
            })?; 
            Ok(())
        }

        /// Runs [SyncComp::drive_abs_async()] for all components
        /// 
        /// # Features
        /// 
        /// Only available if the "std" feature is enabled
        #[cfg(feature = "embed-thread")]
        fn drive_abs_async(&mut self, gamma : [Gamma; C], speed_f : [f32; C]) -> Result<(), crate::Error> {
            self.try_for_each_mut(|comp, index| {
                comp.drive_abs_async(gamma[index], speed_f[index])
            })?;
            Ok(())
        }   

        /// Runs [SyncComp::await_inactive()] for all components
        /// 
        /// # Features
        /// 
        /// Only available if the "std" feature is enabled
        #[cfg(feature = "embed-thread")]
        fn await_inactive(&mut self) -> Result<[Delta; C], crate::Error> {
            self.try_for_each_mut(|comp, _| {
                comp.await_inactive()
            })
        }
    // 

    // Position
        /// Runs [SyncComp::gamma()] for all components
        #[inline(always)]
        fn gammas(&self) -> [Gamma; C] {
            self.for_each(|comp, _| {
                comp.gamma()
            })
        }
        
        /// Runs [SyncComp::write_gamma()] for all components
        #[inline(always)]
        fn write_gammas(&mut self, gammas : &[Gamma; C]) {
            self.for_each_mut(|comp, index| {
                comp.write_gamma(gammas[index])
            });
        }

        /// Runs [SyncComp::lim_for_gamma()] for all components 
        #[inline(always)]
        fn lims_for_gammas(&self, gammas : &[Gamma; C]) -> [Delta; C] {
            self.for_each(|comp, index| {
                comp.lim_for_gamma(gammas[index])
            })
        }

        /// Checks if the given gammas are vaild, which means they are finite and in range of the components
        #[inline(always)]
        fn valid_gammas(&self, gammas : &[Gamma; C]) -> bool {
            self.for_each(|comp, index| {
                !comp.lim_for_gamma(gammas[index]).is_normal() & gammas[index].is_finite()
            }).iter().all(|v| *v)
        }

        /// Same as [SyncCompGroup::valid_gammas()], but it evaluates the check for each component and returns seperated results for analysis
        #[inline(always)]
        fn valid_gammas_verb(&self, gammas : &[Gamma; C]) -> [bool; C] {
            self.for_each(|comp, index| {
                !comp.lim_for_gamma(gammas[index]).is_normal() & gammas[index].is_finite()
            })
        }

        /// Runs [SyncComp::set_end()] for all components
        #[inline(always)]
        fn set_ends(&mut self, set_dist : &[Gamma; C]) {
            self.for_each_mut(|comp, index| {
                comp.set_end(set_dist[index]);
            });
        }
        
        /// Runs [SyncComp::set_limit()] for all components
        #[inline(always)]
        fn set_limits(&mut self, min : &[Option<Gamma>; C], max : &[Option<Gamma>; C]) {
            self.for_each_mut(|comp, index| {
                comp.set_limit(min[index], max[index]);
            }); 
        }
    //

    // Load calculation
        /// Runs [SyncComp::apply_inertia()] for all components
        #[inline(always)]
        fn apply_inertias(&mut self, inertias : &[Inertia; C]) {
            self.for_each_mut(|comp, index| {
                comp.apply_inertia(inertias[index]);
            }); 
        }

        /// Runs [SyncComp::apply_force()] for all components
        #[inline(always)]
        fn apply_forces(&mut self, forces : &[Force; C]) {
            self.for_each_mut(|comp, index| {
                comp.apply_force(forces[index]);
            });
        }

        /// Runs [SyncComp::apply_bend_f()] for all components. Note that the same factor is applied to all components
        #[inline(always)]
        fn apply_bend_f(&mut self, f_bend : f32) {
            self.for_each_mut(|comp, _| {
                comp.apply_bend_f(f_bend);
            }); 
        }

        /// Returns the maximum omegas for each component of the group
        fn omega_max(&self) -> [Omega; C] {
            self.for_each(|comp, _| {
                comp.omega_max()
            })
        }

        /// Set the maximum omega of the components
        fn set_omega_max(&mut self, omega_max : [Omega; C]) {
            self.for_each_mut(|comp, index| {
                comp.set_omega_max(omega_max[index]);
            });
        }
    // 
}

// Implementation
impl<T : SyncComp, const C : usize> Setup for [T; C] {
    fn setup(&mut self) -> Result<(), crate::Error> {
        for i in 0 .. C {
            self[i].setup()?;
        }

        Ok(())
    }
}

impl<T : SyncComp + 'static, const C : usize> SyncCompGroup<T, C> for [T; C] { 
    fn for_each<'a, F, R>(&'a self, mut func : F) -> [R; C]
    where 
        F : FnMut(&'a T, usize) -> R,
        R : Copy + Default 
    {   
        let mut res = [R::default(); C];
        for i in 0 .. C {
            res[i] = func(&self[i], i);
        }
        res
    }

    fn for_each_mut<F, R>(&mut self, mut func : F) -> [R; C]
    where 
        F : FnMut(&mut T, usize) -> R,
        R : Copy + Default 
    {
        let mut res = [R::default(); C];
        for i in 0 .. C {
            res[i] = func(&mut self[i], i);
        }
        res
    }

    fn try_for_each<'a, F, R, E>(&'a self, mut func : F) -> Result<[R; C], E>
    where 
        F : FnMut(&'a T, usize) -> Result<R, E>,
        R : Copy + Default 
    {
        let mut res = [R::default(); C];
        for i in 0 .. C {
            res[i] = func(&self[i], i)?;
        }
        Ok(res)
    }

    fn try_for_each_mut<F, R, E>(&mut self, mut func : F) -> Result<[R; C], E>
    where 
        F : FnMut(&mut T, usize) -> Result<R, E>,
        R : Copy + Default 
    {
        let mut res = [R::default(); C];
        for i in 0 .. C {
            res[i] = func(&mut self[i], i)?;
        }
        Ok(res)
    }

    fn for_each_dyn<'a, F, R>(&'a self, mut func : F) -> Vec<R>
    where 
        F : FnMut(&'a T, usize) -> R 
    {
        let mut res = Vec::with_capacity(C);
        for i in 0 .. C {
            res.push(func(&self[i], i));
        }
        res
    }
}