use crate::math::kin;

use syunit::*;

#[derive(Debug)]
pub struct PathBuilder<const N : usize> {
    times : [Time; N],
    deltas : [Delta; N],

    omega_0 : Velocity,
    omegas_av : [Velocity; N],
    omegas_node : [Velocity; N],

    alphas : [Acceleration; N]
}

impl<const N : usize> PathBuilder<N> {
    pub fn new(times : [Time; N], deltas : [Delta; N], omega_0 : Velocity) -> Self {
        Self {
            times,
            deltas,

            omega_0,
            omegas_av: [Velocity::ZERO; N],
            omegas_node: [Velocity::ZERO; N],

            alphas : [Acceleration::ZERO; N]
        }
    }

    // Indexing
        #[inline]
        pub fn omega_0(&self, i : usize) -> Velocity {
            if i == 0 {
                self.omega_0
            } else {
                self.omegas_node[i - 1]
            }
        }

        #[inline]
        pub fn omega_tar(&self, i : usize) -> Velocity {
            self.omegas_node[i]
        }
    // 

    pub fn gen_omega_av(&mut self) {
        for i in 0 .. N {
            self.omegas_av[i] = self.deltas[i] / self.times[i];
        }
    }

    pub fn update_omega_av(&mut self, i : usize, o : Velocity) {
        if o.abs() > self.omegas_av[i].abs() {
            panic!("Increased omega average! from: {} to: {}", o, self.omegas_av[i]);
        }

        self.omegas_av[i] = o;
        self.times[i] = self.deltas[i] / self.omegas_av[i];
    }

    pub fn gen_omega_nodes(&mut self, omega_end : Velocity) {
        let mut omega_next;
        let mut omega_0 = self.omega_0;
    
        for i in 0 .. N {
            if i < (N - 1) {
                omega_next = self.omegas_av[i + 1];
            } else {
                omega_next = omega_end;
            }
            
            // Change of sign
            if (self.omegas_av[i].0 * omega_next.0) < 0.0 {
                omega_next = Velocity::ZERO;
            }

            let omega_av_n = (omega_next + omega_0) / 2.0;

            if omega_av_n.abs() > self.omegas_av[i].abs() {
                self.omegas_node[i] = 2.0 * self.omegas_av[i] - omega_0;
            } else {
                self.omegas_node[i] = omega_next;
                self.update_omega_av(i, omega_av_n);
            }

            omega_0 = self.omegas_node[i];
        }
    }

    pub fn gen_alphas(&mut self) {
        let mut omega_0 = self.omega_0;

        for i in 0 .. N {
            self.alphas[i] = (self.omegas_node[i] - omega_0) / self.times[i];
            omega_0 = self.omegas_node[i];     
        }
    }

    pub fn check_alpha(&mut self, i : usize, max : Acceleration) {
        // Recalc alpha
        self.alphas[i] = (self.omegas_node[i] - self.omega_0(i)) / self.times[i];   

        if self.alphas[i].abs() > max.abs() {
            let omega_0 = self.omega_0(i);
            let omega_tar = self.omega_tar(i);

            let alpha_use = if self.alphas[i] >= Acceleration::ZERO {
                max.abs()
            } else {
                -max.abs()
            };

            if omega_tar.abs() >= omega_0.abs() {
                // Velocity increasing  (starting point stays fixed because it's smaller)
                if let Some(time) = kin::time::positive_travel_time(self.deltas[i], omega_0, alpha_use) {
                    self.times[i] = time;
                    self.omegas_node[i] = omega_0 + alpha_use * time;
                    self.omegas_av[i] = (omega_0 + self.omegas_node[i]) / 2.0;
                } else {
                    dbg!(&self);
                    panic!("Travel time failed!: D: {}, O_0: {}, A: {}, A_prev: {}", self.deltas[i], omega_0, alpha_use, self.alphas[i]);
                }
            } else {
                // Velocity decreasing (endpoint stays fixed because it's smaller)
                if let Some(time) = kin::time::positive_travel_time(self.deltas[i], omega_0, alpha_use) {
                    self.times[i] = time;
                    self.omegas_node[i - 1] = omega_tar - alpha_use * time;
                    self.omegas_av[i] = (self.omegas_node[i - 1] + self.omegas_node[i]) / 2.0;
                } else {
                    dbg!(&self);
                    panic!("Travel time failed!: D: {}, O_0: {}, A: {}, A_prev: {}", self.deltas[i], omega_0, alpha_use, self.alphas[i]);
                }
            }

            self.alphas[i] = alpha_use;
        }
    }

    pub fn check_all_alphas(&mut self, max : Acceleration) {
        for i in 0 .. N {
            self.check_alpha(i, max);
        }
    }

    pub fn check_all_with_alpha<F : FnMut(&mut Self, usize) -> Acceleration>(&mut self, mut afunc : F) {
        for i in 0 .. N {
            let alpha = afunc(self, i);
            self.check_alpha(i, alpha)
        }
    }
}