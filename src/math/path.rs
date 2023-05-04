use crate::math::CurveBuilder;
use crate::units::*;


#[derive(Debug, Clone)]
pub struct PathNode {
    pub delta : Delta,
    pub omega_0 : Omega,
    pub time : Time
}

impl Default for PathNode {
    fn default() -> Self {
        Self {
            delta: Delta::ZERO,
            omega_0: Omega::INFINITY,
            time: Time::ZERO
        }
    }
}

pub fn create_nodes(path : Vec<Gamma>) -> Vec<PathNode> {
    let mut nodes = vec![PathNode::default(); path.len() - 1]; 

    for i in 0 .. (path.len() - 1) {
        nodes[i].delta = path[i + 1] - path[i];
    }

    nodes
}

#[derive(Debug, Clone)]
pub struct PathBuilder<'a, const N : usize> {
    pub builders : [CurveBuilder<'a>; N],
    pub paths : [Vec<PathNode>; N], 

    pub index : usize
}

impl<'a, const N : usize> PathBuilder<'a, N> {
    pub fn new(builders : [CurveBuilder<'a>; N], paths : [Vec<PathNode>; N]) -> Self {
        Self  {
            builders, paths,
            index: 0
        }
    }

    pub fn calc_point(&mut self, t_min : Time, index : usize) -> f32 {
        let mut t_max = Time::ZERO;
        let mut omegas = [ Omega::INFINITY; N ];

        for i in 0 .. N {
            let ( time, _ ) = self.builders[i].next(
                self.paths[i][index].delta, 
                2.0 * self.paths[i][index].delta / t_min - self.builders[i].omega_0
            );

            if time > t_max {
                for n in 0 .. (i + 1) {
                    omegas[n] = self.builders[n].correct_last(time);
                }

                t_max = time;
            } else {
                omegas[i] = self.builders[i].correct_last(t_max);

                if omegas[i] < Omega::ZERO {
                    return (self.builders[i].omega_0 + omegas[i]) / self.builders[i].omega_0;
                }
            }
        }

        1.0
    } 

    pub fn next(&mut self, t_min : Time) {
        if self.index > self.paths.len() {
            panic!("Path already generated!");
        }

        let f = self.calc_point(t_min, self.index);

        if f == 1.0 {
            for i in 0 .. N {
                self.paths[i][self.index] = self.builders[i].get_node();
            }
        } else {
            for i in 0 .. N {
                
            }
        }

        self.index += 1;

        // omegas
    }

    pub fn correct_back(&mut self, omega : Omega) {

    }
}
