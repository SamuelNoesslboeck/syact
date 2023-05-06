use crate::math::CurveBuilder;
use crate::units::*;

#[derive(Debug, Clone, Copy)]
pub struct PathNode {
    pub delta : Delta,
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

#[derive(Debug, Clone)]
pub struct PathBuilder<'a, const N : usize> {
    builders : [CurveBuilder<'a>; N],
    nstack : Vec<[PathNode; N]>
}

impl<'a, const N : usize> PathBuilder<'a, N> {
    pub fn new(builders : [CurveBuilder<'a>; N]) -> Self {
        Self {
            builders, 
            nstack: vec![ ]
        }
    }

    pub fn next(&mut self, tstack : &mut [Time], dstack : &[[Delta; N]], n : usize, i : usize) {
        let time = tstack[i];
        let delta = dstack[i][n];
        let omega_tar = delta / time;
        let ( time_real, fac ) = self.builders[n].next(delta, omega_tar);
    
        if fac < 1.0 {
            if self.builders[n].was_deccel() {
                let omega_err = self.builders[n].omega - omega_tar;
                let omega_cor = self.builders[n].omega_0 - omega_err;
        
                println!(" => {}: Fac: {}; Correct: {}, Error: {}, Builder: {}", i, fac, omega_cor, omega_err, self.builders[n].omega);
        
                tstack[i - 1] = dstack[i - 1][n] / omega_cor;
    
                for _n in 0 ..=n {
                    self.builders[_n].load_node(&self.nstack[i - 1][_n]);
                    self.next(tstack, dstack, _n, i - 1);
                }
        
                // let (time_res, fac_res) =  builder.next(delta, omega_tar); 
                self.builders[n].next(delta, omega_tar);
            } else {
                if time_real > tstack[i] {
                    tstack[i] = time_real;
                    for _n in 0 .. n {
                        if i == 0 {
                            self.builders[_n].reset();
                        } else {
                            self.builders[_n].load_node(&self.nstack[i - 1][_n]);
                        }
                        self.next(tstack, dstack, _n, i);
                    }
                }
            }
        }
    
        self.nstack[i][n] = self.builders[n].get_node();
        // println!("[{}, {}] Omega_0: {}, Omega: {}, tar: {}", n, i, self.nstack[i][n].omega_0, self.builders[n].omega, omega_tar);
    }

    pub fn generate(&mut self, tstack : &mut [Time], dstack : &[[Delta; N]]) {
        if tstack.len() != dstack.len() {
            panic!("Stacks must be equal in size!");
        }

        self.nstack = vec![ [ PathNode::default(); N ]; tstack.len() ];

        for i in 0 .. tstack.len() {
            for n in 0 .. N {
                self.next(tstack, dstack, n, i);
            }
        }
    }

    pub fn get_node(&'a self, n : usize, i : usize) -> &'a PathNode {
        &self.nstack[i][n]
    }
}
