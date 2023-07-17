use crate::{Setup, SyncComp};
use crate::units::*;

/// A group of synchronous components  
/// This trait allows a lot of functions to be used for all components at once.
pub trait SyncCompGroup<const C : usize> : Setup {
    /// Type that generalizes all componenents, e.g. `dyn SyncComp`
    type Comp : SyncComp;

    // Iteration
        fn for_each<F, R>(&self, func : F) -> [R; C]
        where 
            F : FnMut(&Self::Comp, usize) -> R,
            R : Copy + Default;

        fn for_each_mut<F, R>(&mut self, func : F) -> [R; C]
        where 
            F : FnMut(&mut Self::Comp, usize) -> R,
            R : Copy + Default;

        fn try_for_each<F, R, E>(&self, func : F) -> Result<[R; C], E>
        where 
            F : FnMut(&Self::Comp, usize) -> Result<R, E>,
            R : Copy + Default;

        fn try_for_each_mut<F, R, E>(&mut self, func : F) -> Result<[R; C], E>
        where 
            F : FnMut(&mut Self::Comp, usize) -> Result<R, E>,
            R : Copy + Default;
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
    fn drive_rel(&mut self, deltas : [Delta; C], speed_f : f32) -> Result<[Delta; C], crate::Error> {
        self.try_for_each_mut(|comp, index| {
            comp.drive_rel(deltas[index], speed_f)  
        })
    }

    /// Runs [SyncComp::drive_abs()] for all components
    fn drive_abs(&mut self, gamma : [Gamma; C], speed_f : f32) -> Result<[Delta; C], crate::Error>  {
        self.try_for_each_mut(|comp, index| {
            comp.drive_abs(gamma[index], speed_f)
        })
    }

    // Async
        /// Runs [SyncComp::drive_rel_async()] for all components
        /// 
        /// # Features
        /// 
        /// Only available if the "std" feature is enabled
        #[cfg(feature = "std")]
        fn drive_rel_async(&mut self, deltas : [Delta; C], speed_f : f32) -> Result<(), crate::Error> {
            self.try_for_each_mut(|comp, index| {
                comp.drive_rel_async(deltas[index], speed_f)
            })?; 
            Ok(())
        }

        /// Runs [SyncComp::drive_abs_async()] for all components
        /// 
        /// # Features
        /// 
        /// Only available if the "std" feature is enabled
        #[cfg(feature = "std")]
        fn drive_abs_async(&mut self, gamma : [Gamma; C], speed_f : f32) -> Result<(), crate::Error> {
            self.try_for_each_mut(|comp, index| {
                comp.drive_abs_async(gamma[index], speed_f)
            })?;
            Ok(())
        }   

        /// Runs [SyncComp::await_inactive()] for all components
        /// 
        /// # Features
        /// 
        /// Only available if the "std" feature is enabled
        #[cfg(feature = "std")]
        fn await_inactive(&mut self) -> Result<[Delta; C], crate::Error> {
            self.try_for_each_mut(|comp, index| {
                comp.await_inactive()
            })
        }
    // 

    // Position
        /// Runs [SyncComp::gamma()] for all components
        #[inline(always)]
        fn gammas(&self) -> [Gamma; C] {
            self.for_each(|comp, index| {
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
            let mut limits = [Delta::ZERO; C]; 
            for i in 0 .. C {
                limits[i] = self.index(i).lim_for_gamma(gammas[i]);
            }
            limits
        }

        /// Checks if the given gammas are vaild, which means they are finite and in range of the components
        #[inline(always)]
        fn valid_gammas(&self, gammas : &[Gamma; C]) -> bool {
            let mut res = true;
            for i in 0 .. C {
                res = res & ((!self.index(i).lim_for_gamma(gammas[i]).is_normal()) & gammas[i].is_finite()); 
            }
            res
        }

        /// Same as [SyncCompGroup::valid_gammas()], but it evaluates the check for each component and returns seperated results for analysis
        #[inline(always)]
        fn valid_gammas_verb(&self, gammas : &[Gamma; C]) -> [bool; C] {
            let mut res = [true; C];
            for i in 0 .. C {
                res[i] = (!self.index(i).lim_for_gamma(gammas[i]).is_normal()) & gammas[i].is_finite(); 
            }
            res
        }

        /// Runs [SyncComp::set_end()] for all components
        #[inline(always)]
        fn set_ends(&mut self, set_dist : &[Gamma; C]) {
            for i in 0 .. C {
                self.index_mut(i).set_end(set_dist[i]);
            }
        }
        
        /// Runs [SyncComp::set_limit()] for all components
        #[inline(always)]
        fn set_limits(&mut self, min : &[Option<Gamma>; C], max : &[Option<Gamma>; C]) {
            for i in 0 .. C {
                self.index_mut(i).set_limit(min[i], max[i]);
            }
        }
    //

    // Load calculation
        /// Runs [SyncComp::apply_inertia()] for all components
        #[inline(always)]
        fn apply_inertias(&mut self, inertias : &[Inertia; C]) {
            for i in 0 .. C {
                self.index_mut(i).apply_inertia(inertias[i]);
            }
        }

        /// Runs [SyncComp::apply_force()] for all components
        #[inline(always)]
        fn apply_forces(&mut self, forces : &[Force; C]) {
            for i in 0 .. C {
                self.index_mut(i).apply_force(forces[i]);
            }
        }

        /// Runs [SyncComp::apply_bend_f()] for all components. Note that the same factor is applied to all components
        #[inline(always)]
        fn apply_bend_f(&mut self, f_bend : f32) {
            for i in 0 .. C {
                self.index_mut(i).apply_bend_f(f_bend);
            }
        }

        /// Returns the maximum omegas for each component of the group
        fn omega_max(&self) -> [Omega; C] {
            let mut omegas = [Omega::ZERO; C]; 
            for i in 0 .. C {
                omegas[i] = self.index(i).omega_max()
            }
            omegas
        }

        /// Set the maximum omega of the components
        fn set_omega_max(&mut self, omega_max : [Omega; C]) {
            for i in 0 .. C {
                self.index_mut(i).set_omega_max(omega_max[i])
            }
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

impl<T : SyncComp, const C : usize> SyncCompGroup<C> for [T; C] { 
    type Comp = T;

    fn for_each<F, R>(&self, mut func : F) -> [R; C]
    where 
        F : FnMut(&Self::Comp, usize) -> R,
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
        F : FnMut(&mut Self::Comp, usize) -> R,
        R : Copy + Default 
    {
        let mut res = [R::default(); C];
        for i in 0 .. C {
            res[i] = func(&mut self[i], i);
        }
        res
    }

    fn try_for_each<F, R, E>(&self, mut func : F) -> Result<[R; C], E>
    where 
        F : FnMut(&Self::Comp, usize) -> Result<R, E>,
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
        F : FnMut(&mut Self::Comp, usize) -> Result<R, E>,
        R : Copy + Default 
    {
        let mut res = [R::default(); C];
        for i in 0 .. C {
            res[i] = func(&mut self[i], i)?;
        }
        Ok(res)
    }
}