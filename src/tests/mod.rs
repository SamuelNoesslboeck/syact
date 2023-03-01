#[cfg(target_os = "linux")]
use std::{f32::consts::PI, sync::Arc};

#[cfg(target_os = "linux")]
use gpio::{GpioIn, sysfs::*};

#[cfg(target_os = "linux")]
use crate::{Component, LinkedData, StepperCtrl, StepperConst, UpdateFunc, Delta, Omega};

// Submodules
mod data;

mod movements;

mod servo;
 
mod tools;

// Test Async
    #[test]
    #[cfg(target_os = "linux")]
    fn test_async() {
        let mut ctrl = StepperCtrl::new(StepperConst::MOT_17HE15_1504S, 27, 19);
        ctrl.link(Arc::new(LinkedData { u: 12.0, s_f: 1.5 })); 

        ctrl.comms.send_msg((Delta(4.0 * PI), Omega(2.0 * PI), UpdateFunc::None));

        println!("Msg sent!");
        println!("Awaiting inactive status ...");

        ctrl.comms.await_inactive();
    }
//

// Test Input
    #[test]
    #[cfg(target_os = "linux")]
    fn test_input() {
        let mut pin = SysFsGpioInput::open(25).expect("Could not open pin");
        
        let mut pin_rec = false;

        loop {
            let read_val = pin.read_value().unwrap() == gpio::GpioValue::High;

            if pin_rec && (!read_val) {
                pin_rec = false;

                println!("Input deactivated! {}", read_val);
            } else if (!pin_rec) && read_val {
                pin_rec = true;

                println!("Input activated! {}", read_val);
            }
        }
    }
//

// // Test G1
// mod test_g1 
// {
//     use crate::Inertia;

//     use super::*;

//     const L1 : f32 = 100.0;
//     const L2 : f32 = 100.0;

//     // Helper
//         fn full_atan(x : f32, y : f32) -> f32 {
//             if x == 0.0 {
//                 if y > 0.0 {
//                     return PI / 2.0;
//                 } else if y < 0.0 {
//                     return -(PI / 2.0); 
//                 }

//                 return 0.0;
//             }

//             (y / x).atan() + if x < 0.0 { PI } else { 0.0 }
//         }

//         fn law_of_cosines(a : f32, b : f32, c : f32) -> f32 {
//             ((a.powi(2) + b.powi(2) - c.powi(2)) / 2.0 / a / b).acos()
//         }
//     // 

//     // fn get_pos(comps : &[Box<dyn Component>; 2]) -> Vec2 {
//     //     let [ g_1, g_2 ] = comps.get_dist();
//     //     Vec2::new(
//     //         L1 * g_1.cos() + L2 * (g_1 + g_2).cos(),
//     //         L1 * g_1.sin() + L2 * (g_1 + g_2).sin()
//     //     )
//     // }

//     fn get_angles(pos : Vec2) -> [f32; 2] {
//         let length = (pos[0].powi(2) + pos[1].powi(2)).powf(0.5);
//         let angle = full_atan(pos[0], pos[1]);
        
//         let gamma = law_of_cosines(L1, L2, length);
//         let alpha = law_of_cosines(L1, length, L2);

//         [
//             angle + alpha,
//             gamma - PI
//         ]
//     }

//     fn vectors_for_angles(angles : [f32; 2]) -> [Vec2; 2] {
//         [
//             Mat2::from_angle(angles[0]) * Vec2::new(L1, 0.0),
//             Mat2::from_angle(angles[0] + angles[1]) * Vec2::new(L2, 0.0)
//         ]
//     }

//     // fn points_for_angles(angles : [f32; 2]) -> [Vec2; 2] {
//     //     let [ a_1, a_2 ] = vectors_for_angles(angles); 
//     //     [ a_1, a_1 + a_2 ]
//     // }

//     fn actor_vecs(angles : [f32; 2]) -> [Vec2; 2] {
//         let [ a_1, a_2 ] = vectors_for_angles(angles); 
//         [ Mat2::from_angle(PI / 2.0) * (a_1 + a_2), Mat2::from_angle(PI / 2.0) * a_2 ]
//     }

//     fn relevance(actors : &[Vec2; 2], vel : Vec2) -> Vec2 {
//         let mut relev = Mat2::from_cols(actors[0], actors[1]).inverse() * vel;
//         if relev.x.abs() < f32::EPSILON {
//             relev.x = 0.0;
//         }

//         if relev.y.abs() < f32::EPSILON {
//             relev.y = 0.0;
//         }

//         relev
//     }   

//     fn get_lin_move(pos_0 : Vec2, pos : Vec2, vel_max : f32, n_seg : usize) -> CompPath<2> {
//         let delta_pos = pos - pos_0;
//         let delta_seg = delta_pos / (n_seg as f32);

//         let mut phis = vec![];
//         let mut relev = vec![];

//         for i in 0 .. (n_seg + 1) {
//             let current_pos = pos_0 + delta_seg * (i as f32);

//             let angles = get_angles(current_pos);
//             let actors = actor_vecs(angles);

//             relev.push(relevance(&actors, delta_pos.normalize() * vel_max).to_array());
//             phis.push(angles);
//         }

//         CompPath::new(phis, relev)
//     }

//     #[test]
//     fn test_g1() {
//         const U : f32 = 12.0;
//         const SF : f32 = 1.5;

//         let mut comps : [Box<dyn Component>; 2] = [ 
//             Box::new(StepperCtrl::new(StepperConst::MOT_17HE15_1504S, PIN_ERR, PIN_ERR)),
//             Box::new(StepperCtrl::new(StepperConst::MOT_17HE15_1504S, PIN_ERR, PIN_ERR))
//         ]; 

//         comps.link_all(Arc::new(LinkedData { u: U, s_f: SF }));
//         comps.apply_load_inertias(&[Inertia(0.05), Inertia(0.05)]);

//         // dbg!(comps[0].accel_max_node(0.0, 0.0, 0.5, 10.0));
//         // dbg!(comps[0].compl_times(0.0, 0.0, 0.5, 10.0));

//         let mut path = get_lin_move(Vec2::new(50.0, 100.0), Vec2::new(150.0, -50.0), 1000.0, 50);

//         path.generate(&comps, [0.0, 0.0], [0.0, 0.0], 0.98);

//         // path.debug_path(0);
//         // dbg!(path.phis);
//         dbg!(path.omegas);
//     }
// }
// // 

// // Test Simple G1
// mod test_simple_g1 
// {
//     use crate::Inertia;

//     use super::*;

//     // const L1 : f32 = 100.0;

//     // Helper
//         fn full_atan(x : f32, y : f32) -> f32 {
//             if x == 0.0 {
//                 if y > 0.0 {
//                     return PI / 2.0;
//                 } else if y < 0.0 {
//                     return -(PI / 2.0); 
//                 }

//                 return 0.0;
//             }

//             (y / x).atan() + if x < 0.0 { PI } else { 0.0 }
//         }

//         // fn law_of_cosines(a : f32, b : f32, c : f32) -> f32 {
//         //     ((a.powi(2) + b.powi(2) - c.powi(2)) / 2.0 / a / b).acos()
//         // }
//     // 

//     // fn get_pos(comps : &[Box<dyn Component>; 2]) -> Vec2 {
//     //     let [ g_1, g_2 ] = comps.get_dist();
//     //     Vec2::new(
//     //         L1 * g_1.cos(),
//     //         L1 * g_1.sin()
//     //     )
//     // }

//     fn get_angles(pos : Vec2) -> [f32; 1] {
//         let angle = full_atan(pos[0], pos[1]);

//         [ angle ]
//     }

//     // fn vectors_for_angles(angles : [f32; 1]) -> [Vec2; 1] {
//     //     [
//     //         Mat2::from_angle(angles[0]) * Vec2::new(L1, 0.0)
//     //     ]
//     // }

//     // fn points_for_angles(angles : [f32; 2]) -> [Vec2; 2] {
//     //     let [ a_1, a_2 ] = vectors_for_angles(angles); 
//     //     [ a_1, a_1 + a_2 ]
//     // }

//     // fn actor_vecs(angles : [f32; 1]) -> [Vec2; 1] {
//     //     let [ a_1 ] = vectors_for_angles(angles); 
//     //     [ Mat2::from_angle(PI / 2.0) * a_1 ]
//     // }

//     fn relevance(pos : [f32; 2], vel : [f32; 2]) -> [f32; 1] {
//         let x = vel[1] / pos[0];
//         let y = vel[0] / pos[1];

//         [ if x.is_finite() { x } else { 0.0 } - if y.is_finite() { -y } else { -0.0 } ]
//     }   

//     fn get_lin_move(pos_0 : Vec2, pos : Vec2, vel_max : f32, n_seg : usize) -> CompPath<1> {
//         let delta_pos = pos - pos_0;
//         let delta_seg = delta_pos / (n_seg as f32);

//         let mut phis = vec![];
//         let mut relev = vec![];

//         for i in 0 .. (n_seg + 1) {
//             let current_pos = pos_0 + delta_seg * (i as f32);

//             let angles = get_angles(current_pos);

//             relev.push(relevance(current_pos.to_array(), (delta_pos.normalize() * vel_max).to_array()));
//             phis.push(angles);
//         }

//         CompPath::new(phis, relev)
//     }

//     #[test]
//     fn test_simple_g1() {
//         const U : f32 = 12.0;
//         const SF : f32 = 1.5;

//         let mut comps : [Box<dyn Component>; 1] = [ 
//             Box::new(StepperCtrl::new(StepperConst::MOT_17HE15_1504S, PIN_ERR, PIN_ERR))
//         ]; 

//         comps.link_all(Arc::new(LinkedData { u: U, s_f: SF }));
//         comps.apply_load_inertias(&[ Inertia(0.5) ]);

//         let mut path = get_lin_move(Vec2::new(50.0, 50.0), Vec2::new(50.0, -50.0), 50.0, 10);

//         path.generate(&comps, [ 0.0 ], [ 0.0 ], 0.95);

//         dbg!(path.omegas);
//     }
// }
// // 