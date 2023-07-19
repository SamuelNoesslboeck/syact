

use crate::math::CurveBuilder;
use crate::units::*;

/// A node for describing a path driven by a stepper motor
#[derive(Debug, Clone, Copy)]
pub struct PathNode {
    /// Delta distance covered in the node
    pub delta : Delta,
    /// Start velocity of the node
    pub omega_0 : Omega
}

impl Default for PathNode {
    fn default() -> Self {
        Self {
            delta: Delta::ZERO,
            omega_0: Omega::INFINITY
        }
    }
}

/// A structure that helps building paths for multiple stepper motors
#[derive(Debug, Clone)]
pub struct PathBuilder<'a, const N : usize> {
    builders : [CurveBuilder<'a>; N],
    nstack : Vec<[PathNode; N]>
}

impl<'a, const N : usize> PathBuilder<'a, N> {
    /// Create a new pathbuilder from a set of curvebuilders
    pub fn new(builders : [CurveBuilder<'a>; N]) -> Self {
        Self {
            builders, 
            nstack: vec![ ]
        }
    }

    /// Iterate through the path
    pub fn next(&mut self, tstack : &mut [Time], dstack : &[[Delta; N]], omega_tar_opt : Option<Omega>, n : usize, i : usize) {
        let time = tstack[i];
        let delta = dstack[i][n];
        let omega_tar = omega_tar_opt.unwrap_or(delta / time);
        let ( time_real, fac ) = self.builders[n].next(delta, omega_tar, None);
    
        if (fac - 1.0) < -0.005 {               // Hysteresis
            if self.builders[n].was_deccel() {
                let omega_err = self.builders[n].omega - omega_tar;
                let omega_cor = self.builders[n].omega_0 - omega_err;
        
                // println!(" => {}: Fac: {}; Correct: {}, Error: {}, Builder: {}", i, fac, omega_cor, omega_err, self.builders[n].omega);
        
                tstack[i - 1] = dstack[i - 1][n] / omega_cor;
    
                for _n in 0 ..=n {
                    self.builders[_n].load_node(&self.nstack[i - 1][_n]);
                    self.next(tstack, dstack, None, _n, i - 1);
                }
        
                // let (time_res, fac_res) =  builder.next(delta, omega_tar); 
                self.builders[n].next(delta, omega_tar, None);
            } else {
                // println!(" => {}: Fac: {}; Builder: {}", i, fac, self.builders[n].omega);

                if time_real > tstack[i] {
                    tstack[i] = time_real;
                    for _n in 0 .. n {
                        if i == 0 {
                            self.builders[_n].reset();
                        } else {
                            self.builders[_n].load_node(&self.nstack[i - 1][_n]);
                        }
                        self.next(tstack, dstack, None, _n, i);
                    }
                }
            }
        }
    
        self.nstack[i][n] = self.builders[n].get_node();
        // println!("[i: {}, n: {}] Omega_0: {}, Omega: {}, tar: {}", i, n, self.nstack[i][n].omega_0, self.builders[n].omega, omega_tar);
    }

    /// Iterate a full row through the path
    pub fn next_all(&mut self, tstack : &mut [Time], dstack : &[[Delta; N]], omega_tar_opt : [Option<Omega>; N], i : usize) {
        for n in 0 .. N {
            self.next(tstack, dstack, omega_tar_opt[n], n, i);
        }
    }

    /// Generate the complete path
    pub fn generate(&mut self, tstack : &mut [Time], dstack : &[[Delta; N]], omega_last : [Option<Omega>; N]) {
        if tstack.len() != dstack.len() {
            panic!("Stacks must be equal in size!");
        }

        self.nstack = vec![ [ PathNode::default(); N ]; tstack.len() ];

        for i in 0 .. (tstack.len() - 1) {
            for n in 0 .. N {
                self.next(tstack, dstack, None, n, i);
            }
        }

        for n in 0 .. N {
            self.next(tstack, dstack, omega_last[n], n, tstack.len() - 1);
        }
    }

    /// Get a pathnode stored in the builder
    pub fn get_node(&'a self, n : usize, i : usize) -> &'a PathNode {
        &self.nstack[i][n]
    }

    /// Consumes the builder, returning it's nodes
    pub fn unpack(self) -> Vec<[PathNode; N]> {
        self.nstack
    }
}
