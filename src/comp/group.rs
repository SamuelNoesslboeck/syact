use crate::{Setup, SyncComp};
use crate::units::*;

/// A group of synchronous components that can be implemented for any type of array, vector or list as long as it can be indexed.
/// This trait then allows a lot of functions to be used to execute functions for all components at once.
pub trait SyncCompGroup<const C : usize> : Setup {
    // Index 
        /// Returns the component at the given index
        fn index<'a>(&'a self, index : usize) -> &'a dyn SyncComp;

        /// Returns the component at the given index
        fn index_mut<'a>(&'a mut self, index : usize) -> &'a mut dyn SyncComp;
    //

    // Data
        /// Runs [SyncComp::write_link()] for all components in the group. Note that the function is using the same 
        /// [LinkedData](crate::data::LinkedData) for all components
        fn write_link(&mut self, lk : crate::data::LinkedData) {
            for i in 0 .. C {
                self.index_mut(i).write_link(lk.clone())
            }
        } 
    //

    /// Runs [SyncComp::drive_rel()] for all components
    fn drive_rel(&mut self, deltas : [Delta; C], speed_f : f32) -> Result<[Delta; C], crate::Error> {
        let mut res = [Delta::ZERO; C];
        for i in 0 .. C {
            res[i] = self.index_mut(i).drive_rel(deltas[i], speed_f)?;
        }
        Ok(res)
    }

    /// Runs [SyncComp::drive_abs()] for all components
    fn drive_abs(&mut self, gamma : [Gamma; C], speed_f : f32) -> Result<[Delta; C], crate::Error>  {
        let mut res = [Delta::ZERO; C];
        for i in 0 .. C {
            res[i] = self.index_mut(i).drive_abs(gamma[i], speed_f)?;
        }
        Ok(res)
    }

    // Async
        /// Runs [SyncComp::drive_rel_async()] for all components
        /// 
        /// # Features
        /// 
        /// Only available if the "std" feature is enabled
        #[cfg(feature = "std")]
        fn drive_rel_async(&mut self, deltas : [Delta; C], speed_f : f32) -> Result<(), crate::Error> {
            for i in 0 .. C {
                self.index_mut(i).drive_rel_async(deltas[i], speed_f)?;
            }
            Ok(())
        }

        /// Runs [SyncComp::drive_abs_async()] for all components
        /// 
        /// # Features
        /// 
        /// Only available if the "std" feature is enabled
        #[cfg(feature = "std")]
        fn drive_abs_async(&mut self, gamma : [Gamma; C], speed_f : f32) -> Result<(), crate::Error> {
            for i in 0 .. C {
                self.index_mut(i).drive_abs_async(gamma[i], speed_f)?;
            }
            Ok(())
        }   

        /// Runs [SyncComp::await_inactive()] for all components
        /// 
        /// # Features
        /// 
        /// Only available if the "std" feature is enabled
        #[cfg(feature = "std")]
        fn await_inactive(&mut self) -> Result<[Delta; C], crate::Error> {
            let mut delta = [Delta::NAN; C];

            for i in 0 .. C {
                delta[i] = self.index_mut(i).await_inactive()?;
            }

            Ok(delta)
        }
    // 

    // Position
        /// Runs [SyncComp::gamma()] for all components
        #[inline(always)]
        fn gammas(&self) -> [Gamma; C] {
            let mut dists = [Gamma::ZERO; C];
            for i in 0 .. C {
                dists[i] = self.index(i).gamma();
            }
            dists
        }
        
        /// Runs [SyncComp::write_gamma()] for all components
        #[inline(always)]
        fn write_gammas(&mut self, gammas : &[Gamma; C]) {
            for i in 0 .. C {
                self.index_mut(i).write_gamma(gammas[i])
            }
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
    #[inline]
    fn index<'a>(&'a self, index : usize) -> &'a dyn SyncComp {
        &self[index]
    }

    #[inline]
    fn index_mut<'a>(&'a mut self, index : usize) -> &'a mut dyn SyncComp {
        &mut self[index]
    }
}