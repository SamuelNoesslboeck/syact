use glam::{Vec2, Mat2};
use gpio::{GpioIn, sysfs::*};
use gcode::{Mnemonic, GCode};
use crate::{Component, StepperCtrl, StepperData, UpdateFunc, gcode::{Interpreter, GCodeFunc, Args}, ctrl::{PIN_ERR, CompPath}, math::actors, ComponentGroup};
use std::{f32::consts::PI, collections::HashMap};
 
// Test Async
    #[test]
    fn test_async() {
        let ctrl = StepperCtrl::new(StepperData::mot_17he15_1504s(12.0, 1.5), 27, 19);
        ctrl.comms.send_msg((4.0 * PI, 2.0 * PI, UpdateFunc::None));

        println!("Msg sent!");
        println!("Awaiting inactive status ...");

        ctrl.comms.await_inactive();
    }
//


// Test Gcode
    struct Data 
    {
        pub pos : f64
    }

    fn g_0(data : &mut Data, _gc : &GCode, _args : &Args) -> Option<()> {
        data.pos += 10.0;
        // intpr.log_ln("G0 function executed");
        None
    }

    fn g_1(data : &mut Data, _gc : &GCode, _args : &Args) -> Option<()> {
        data.pos -= 5.0;
        None
    }

    #[test]
    fn test_gcode() {
        let map = HashMap::from([
            ( Mnemonic::General, HashMap::from([
                ( 0u32, g_0 as GCodeFunc<Data, Option<()>> ),
                ( 1u32, g_1 as GCodeFunc<Data, Option<()>> )
            ]) )
        ]);

        let mut intpr = Interpreter::new(Data { pos: 0.0 }, map);

        let res = intpr.interpret("G0\nG1", |_| { Some(()) });
        dbg!(res);
    }
// 

// Test Input
    #[test]
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

// Test step
    #[test]
    fn test_step() {
        let mut ctrl = StepperCtrl::new(
            StepperData::mot_17he15_1504s(12.0, 1.5), 
            27, 19
        );

        ctrl.apply_load_inertia(0.000_1);

        // Test
        println!("Doing single step ... ");
        ctrl.step(0.01, &crate::UpdateFunc::None);
        println!("Step done ... ");
        // 
    }
//

// Test steps
    // Parameters
        const STEPS : u64 = 200;
        const OMEGA : f32 = 10.0;
    // 

    #[test]
    fn test_steps() {
        let mut ctrl = StepperCtrl::new(
            StepperData::mot_17he15_1504s(12.0, 1.5), 
            27, 19);

        ctrl.apply_load_inertia(0.1);
        ctrl.apply_load_force(0.1);

        println!("Staring to move");
        ctrl.steps(STEPS, OMEGA, crate::ctrl::UpdateFunc::None);
        println!("{} with max speed {}rad/s done", STEPS, OMEGA);
    }
// 

// Test G1
    const L1 : f32 = 100.0;
    const L2 : f32 = 100.0;

    // Helper
        fn full_atan(x : f32, y : f32) -> f32 {
            if x == 0.0 {
                if y > 0.0 {
                    return PI / 2.0;
                } else if y < 0.0 {
                    return -(PI / 2.0); 
                }

                return 0.0;
            }

            (y / x).atan()
        }

        fn law_of_cosines(a : f32, b : f32, c : f32) -> f32 {
            ((a.powi(2) + b.powi(2) - c.powi(2)) / 2.0 / a / b).acos()
        }
    // 

    fn get_pos(comps : &[Box<dyn Component>; 2]) -> Vec2 {
        let [ g_1, g_2 ] = comps.get_dist();
        Vec2::new(
            L1 * g_1.cos() + L2 * (g_1 + g_2).cos(),
            L1 * g_1.sin() + L2 * (g_1 + g_2).sin()
        )
    }

    fn get_angles(pos : Vec2) -> [f32; 2] {
        let length = (pos[0].powi(2) + pos[1].powi(2)).powf(0.5);
        let angle = full_atan(pos[0], pos[1]);
        
        let gamma = law_of_cosines(L1, L2, length);
        let alpha = law_of_cosines(L1, length, L2);

        [
            angle + alpha,
            gamma - PI
        ]
    }

    fn vectors_for_angles(angles : [f32; 2]) -> [Vec2; 2] {
        [
            Mat2::from_angle(angles[0]) * Vec2::new(L1, 0.0),
            Mat2::from_angle(angles[0] + angles[1]) * Vec2::new(L2, 0.0)
        ]
    }

    fn points_for_angles(angles : [f32; 2]) -> [Vec2; 2] {
        let [ a_1, a_2 ] = vectors_for_angles(angles); 
        [ a_1, a_1 + a_2 ]
    }

    fn actor_vecs(angles : [f32; 2]) -> [Vec2; 2] {
        let [ a_1, a_2 ] = vectors_for_angles(angles); 
        [ Mat2::from_angle(PI / 2.0) * a_1, Mat2::from_angle(PI / 2.0) * a_2 ]
    }

    fn relevance(actors : &[Vec2; 2], vel : Vec2) -> Vec2 {
        let mut relev = (Mat2::from_cols(actors[0], actors[1]).inverse() * vel).abs();
        if relev.x < f32::EPSILON {
            relev.x = 0.0;
        }

        if relev.y < f32::EPSILON {
            relev.y = 0.0;
        }

        relev
    }   

    fn get_lin_move(pos_0 : Vec2, pos : Vec2, vel_max : f32, n_seg : usize) -> CompPath<2> {
        let delta_pos = pos - pos_0;
        let delta_seg = delta_pos / (n_seg as f32);

        let mut phis = vec![];
        let mut relev = vec![];

        for i in 0 .. (n_seg + 1) {
            let current_pos = pos_0 + delta_seg * (i as f32);

            let angles = get_angles(current_pos);
            let actors = actor_vecs(angles);

            relev.push(relevance(&actors, delta_pos.normalize() * vel_max).to_array());
            phis.push(angles);
        }

        CompPath::new(phis, relev)
    }

    #[test]
    fn test_g1() {
        const U : f32 = 12.0;
        const SF : f32 = 1.5;

        let comps : [Box<dyn Component>; 2] = [ 
            Box::new(StepperCtrl::new(StepperData::mot_17he15_1504s(U, SF), PIN_ERR, PIN_ERR)),
            Box::new(StepperCtrl::new(StepperData::mot_17he15_1504s(U, SF), PIN_ERR, PIN_ERR))
        ]; 

        comps.apply_load_inertia([0.001; 2]);

        let mut path = get_lin_move(Vec2::new(100.0, 100.0), Vec2::new(-100.0, 100.0), 50.0, 10);

        // dbg!(&path.relev);

        path.generate(&comps, [0.0, 0.0], [0.0, 0.0], 50.0);

        dbg!(path.omegas);
    }
// 