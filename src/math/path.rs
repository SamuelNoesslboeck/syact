use crate::math::kin;

use syunit::*;

#[derive(Debug)]
pub struct PathBuilder<const N : usize> {
    times : [Time; N],
    deltas : [Delta; N],

    velocity_0 : Velocity,
    velocitys_av : [Velocity; N],
    velocitys_node : [Velocity; N],

    alphas : [Acceleration; N]
}

impl<const N : usize> PathBuilder<N> {
    pub fn new(times : [Time; N], deltas : [Delta; N], velocity_0 : Velocity) -> Self {
        Self {
            times,
            deltas,

            velocity_0,
            velocitys_av: [Velocity::ZERO; N],
            velocitys_node: [Velocity::ZERO; N],

            alphas : [Acceleration::ZERO; N]
        }
    }

    // Indexing
        #[inline]
        pub fn velocity_0(&self, i : usize) -> Velocity {
            if i == 0 {
                self.velocity_0
            } else {
                self.velocitys_node[i - 1]
            }
        }

        #[inline]
        pub fn velocity_tar(&self, i : usize) -> Velocity {
            self.velocitys_node[i]
        }
    // 

    pub fn gen_velocity_av(&mut self) {
        for i in 0 .. N {
            self.velocitys_av[i] = self.deltas[i] / self.times[i];
        }
    }

    pub fn update_velocity_av(&mut self, i : usize, o : Velocity) {
        if o.abs() > self.velocitys_av[i].abs() {
            panic!("Increased velocity  average! from: {} to: {}", o, self.velocitys_av[i]);
        }

        self.velocitys_av[i] = o;
        self.times[i] = self.deltas[i] / self.velocitys_av[i];
    }

    pub fn gen_velocity_nodes(&mut self, velocity_end : Velocity) {
        let mut velocity_next;
        let mut velocity_0 = self.velocity_0;
    
        for i in 0 .. N {
            if i < (N - 1) {
                velocity_next = self.velocitys_av[i + 1];
            } else {
                velocity_next = velocity_end;
            }
            
            // Change of sign
            if (self.velocitys_av[i].0 * velocity_next.0) < 0.0 {
                velocity_next = Velocity::ZERO;
            }

            let velocity_av_n = (velocity_next + velocity_0) / 2.0;

            if velocity_av_n.abs() > self.velocitys_av[i].abs() {
                self.velocitys_node[i] = 2.0 * self.velocitys_av[i] - velocity_0;
            } else {
                self.velocitys_node[i] = velocity_next;
                self.update_velocity_av(i, velocity_av_n);
            }

            velocity_0 = self.velocitys_node[i];
        }
    }

    pub fn gen_alphas(&mut self) {
        let mut velocity_0 = self.velocity_0;

        for i in 0 .. N {
            self.alphas[i] = (self.velocitys_node[i] - velocity_0) / self.times[i];
            velocity_0 = self.velocitys_node[i];     
        }
    }

    pub fn check_alpha(&mut self, i : usize, max : Acceleration) {
        // Recalc alpha
        self.alphas[i] = (self.velocitys_node[i] - self.velocity_0(i)) / self.times[i];   

        if self.alphas[i].abs() > max.abs() {
            let velocity_0 = self.velocity_0(i);
            let velocity_tar = self.velocity_tar(i);

            let alpha_use = if self.alphas[i] >= Acceleration::ZERO {
                max.abs()
            } else {
                -max.abs()
            };

            if velocity_tar.abs() >= velocity_0.abs() {
                // Velocity increasing  (starting point stays fixed because it's smaller)
                if let Some(time) = kin::time::positive_travel_time(self.deltas[i], velocity_0, alpha_use) {
                    self.times[i] = time;
                    self.velocitys_node[i] = velocity_0 + alpha_use * time;
                    self.velocitys_av[i] = (velocity_0 + self.velocitys_node[i]) / 2.0;
                } else {
                    dbg!(&self);
                    panic!("Travel time failed!: D: {}, O_0: {}, A: {}, A_prev: {}", self.deltas[i], velocity_0, alpha_use, self.alphas[i]);
                }
            } else {
                // Velocity decreasing (endpoint stays fixed because it's smaller)
                if let Some(time) = kin::time::positive_travel_time(self.deltas[i], velocity_0, alpha_use) {
                    self.times[i] = time;
                    self.velocitys_node[i - 1] = velocity_tar - alpha_use * time;
                    self.velocitys_av[i] = (self.velocitys_node[i - 1] + self.velocitys_node[i]) / 2.0;
                } else {
                    dbg!(&self);
                    panic!("Travel time failed!: D: {}, O_0: {}, A: {}, A_prev: {}", self.deltas[i], velocity_0, alpha_use, self.alphas[i]);
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