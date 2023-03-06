// Units
use core::ops::{Add, Sub, Div, Mul, Neg, AddAssign};
use core::time::Duration;

use serde::{Serialize, Deserialize}; 

macro_rules! additive_unit {
    ( $unit:ident ) => {
        impl Add<$unit> for $unit {
            type Output = $unit;
        
            #[inline(always)]
            fn add(self, rhs: $unit) -> Self::Output {
                $unit(self.0 + rhs.0)
            }
        }

        impl AddAssign<$unit> for $unit {
            #[inline(always)]
            fn add_assign(&mut self, rhs: $unit) {
                self.0 += rhs.0;
            }
        }        
        
        impl Sub<$unit> for $unit {
            type Output = $unit;
        
            #[inline(always)]
            fn sub(self, rhs: $unit) -> Self::Output {
                $unit(self.0 - rhs.0) 
            }
        }
    };
}

macro_rules! basic_unit {
    ( $a:ident ) => {      
        impl $a {
            pub const ZERO : Self = Self(0.0);
            pub const INFINITY : Self = Self(f32::INFINITY);
            pub const NAN : Self = Self(f32::NAN);

            /// Returns the absolute value of the unit
            pub fn abs(self) -> Self {
                Self(self.0.abs())
            }

            pub fn is_finite(self) -> bool {
                self.0.is_finite()
            }

            pub fn is_normal(self) -> bool {
                self.0.is_normal()
            }

            pub fn is_nan(self) -> bool {
                self.0.is_nan()
            }

            pub fn sin(self) -> f32 {
                self.0.sin()
            }
        }

        impl Into<f32> for $a {
            #[inline(always)]
            fn into(self) -> f32 {
                self.0
            }
        }

        // Implementing ordering traits
            impl Eq for $a { }
            
            impl Ord for $a {
                #[inline(always)]
                fn cmp(&self, other: &Self) -> core::cmp::Ordering {
                    self.0.total_cmp(&other.0)
                }
            }
        //

        // Negation
            impl Neg for $a {
                type Output = Self;
                
                #[inline(always)]
                fn neg(self) -> Self::Output {
                    Self(-self.0)
                }
            }
        //

        // Multiplication
            impl Mul<f32> for $a {
                type Output = $a;
                
                #[inline(always)]
                fn mul(self, rhs: f32) -> Self::Output {
                    $a(self.0 * rhs)
                }
            }

            impl Mul<$a> for f32 {
                type Output = $a;

                #[inline(always)]
                fn mul(self, rhs : $a) -> Self::Output {
                    $a(self * rhs.0)
                }
            }
        // 
        
        // Division
            impl Div<f32> for $a {
                type Output = $a;
            
                #[inline(always)]
                fn div(self, rhs: f32) -> Self::Output {
                    $a(self.0 / rhs)
                }
            }

            impl Div<$a> for $a {
                type Output = f32;

                #[inline(always)]
                fn div(self, rhs : $a) -> Self::Output {
                    self.0 / rhs.0
                }
            }
        // 
    };
}

macro_rules! derive_units {
    ( $dist:ident, $vel:ident, $time:ident ) => {
        impl Mul<$time> for $vel {
            type Output = $dist;
        
            #[inline(always)]
            fn mul(self, rhs: $time) -> Self::Output {
                $dist(self.0 / rhs.0)
            }
        }

        impl Mul<$vel> for $time {
            type Output = $dist;
        
            #[inline(always)]
            fn mul(self, rhs: $vel) -> Self::Output {
                $dist(self.0 / rhs.0)
            }
        }

        impl Div<$vel> for $dist {
            type Output = $time;
        
            #[inline(always)]
            fn div(self, rhs: $vel) -> Self::Output {
                $time(self.0 / rhs.0)
            }
        }

        impl Div<$time> for $dist {
            type Output = $vel;
        
            #[inline(always)]
            fn div(self, rhs: $time) -> Self::Output {
                $vel(self.0 / rhs.0)
            }
        }
    };
}

/// Represents a time
/// 
/// # Unit
/// 
/// - In seconds
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq, PartialOrd)]
pub struct Time(pub f32);
basic_unit!(Time);
additive_unit!(Time);

impl Into<Duration> for Time {
    #[inline(always)]
    fn into(self) -> Duration {
        Duration::from_secs_f32(self.0) 
    }
}

impl Div<Time> for f32 {
    type Output = Omega;

    #[inline(always)]
    fn div(self, rhs: Time) -> Self::Output {
        Omega(self / rhs.0)
    }
}

/// The gamma distance represents the actual distance traveled by the component
/// 
/// # Unit
/// 
/// - Can be either radians or millimeters
/// 
//// # Operations
/// ```rust
/// use stepper_lib::units::{Gamma, Delta};
/// 
/// // Subtract two absolute distances to get once relative
/// assert_eq!(Gamma(2.0) - Gamma(1.0), Delta(1.0));
/// 
/// // Add relative distance to absolute one
/// assert_eq!(Gamma(2.0) + Delta(1.0), Gamma(3.0));
/// 
/// assert_eq!(Gamma(2.0) - Delta(1.0), Gamma(1.0));
/// ```
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq, PartialOrd)]
pub struct Gamma(pub f32);
basic_unit!(Gamma);

impl Gamma {
    pub fn force_to_phi(self) -> Phi {
        Phi(self.0)
    }
}

impl Sub<Gamma> for Gamma {
    type Output = Delta;
    
    #[inline(always)]
    fn sub(self, rhs: Gamma) -> Self::Output {
        Delta(self.0 - rhs.0)
    }
}

impl Add<Delta> for Gamma {
    type Output = Gamma;

    #[inline(always)]
    fn add(self, rhs: Delta) -> Self::Output {
        Self(self.0 + rhs.0)
    }
}

impl Sub<Delta> for Gamma {
    type Output = Gamma;

    #[inline(always)]
    fn sub(self, rhs: Delta) -> Self::Output {
        Self(self.0 - rhs.0)
    }
}

#[inline]
pub fn force_phis_from_gammas<const N : usize>(gammas : [Gamma; N]) -> [Phi; N] {
    let mut phis = [Phi::ZERO; N];
    for i in 0 .. N {
        phis[i] = gammas[i].force_to_phi();
    }
    phis
}

#[inline]
pub fn force_gammas_from_phis<const N : usize>(phis : [Phi; N]) -> [Gamma; N] {
    let mut gammas = [Gamma::ZERO; N];
    for i in 0 .. N {
        gammas[i] = phis[i].force_to_gamma();
    }
    gammas
}

/// The phi distance represents the mathematical distance used for calculations
/// 
/// # Unit
/// 
/// - Can be either radians or millimeters
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq, PartialOrd)]
pub struct Phi(pub f32);
basic_unit!(Phi);

impl Phi {
    pub fn force_to_gamma(self) -> Gamma {
        Gamma(self.0)
    }
}

/// The delta distance represents a relative distance traveled by the 
/// 
/// # Unit
/// 
/// - Can be either radians or millimeters
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq, PartialOrd)]
pub struct Delta(pub f32);
basic_unit!(Delta);
additive_unit!(Delta);

impl Delta {
    /// Creates a new delta distance from a starting point `start` and an endpoint `end`
    #[inline(always)]
    pub fn diff(start : Gamma, end : Gamma) -> Self {
        end - start
    }
}

impl Add<Gamma> for Delta {
    type Output = Gamma;

    #[inline(always)]
    fn add(self, rhs: Gamma) -> Self::Output {
        Gamma(self.0 + rhs.0)
    }
}

/// Represents a change rate in distance
/// 
/// # Unit
/// 
/// - Can be either radians per second or millimeters per second
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq, PartialOrd)]
pub struct Omega(pub f32);
basic_unit!(Omega);
additive_unit!(Omega);
derive_units!(Delta, Omega, Time);

impl Div<Omega> for f32 {
    type Output = Time;

    #[inline(always)]
    fn div(self, rhs: Omega) -> Self::Output {
        Time(self / rhs.0)
    }
}

/// Represents a change rate in velocity
/// 
/// # Unit
/// 
/// - Can be either radians per second^2 or millimeters per second^2
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq, PartialOrd)]
pub struct Alpha(pub f32); 
basic_unit!(Alpha);
additive_unit!(Alpha);
derive_units!(Omega, Alpha, Time);
derive_units!(Force, Alpha, Inertia);

/// Represents an inertia, slowing down movement processes
/// 
/// # Unit
/// 
/// - Can be either kilogramm or kilogramm times meter^2
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq, PartialOrd)]
pub struct Inertia(pub f32);
basic_unit!(Inertia);
additive_unit!(Inertia);

/// Represents a force, slowing down movement processes, eventually even overloading the component
/// 
/// 
/// # Unit
/// 
/// - Can be either Newton or Newtonmeter
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq, PartialOrd)]
pub struct Force(pub f32);
basic_unit!(Force);
additive_unit!(Force);