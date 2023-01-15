use gpio::{GpioIn, sysfs::*};
use gcode::{Mnemonic, GCode};
use crate::{Component, StepperCtrl, StepperData, UpdateFunc, gcode::{Interpreter, GCodeFunc, Args}, ctrl::PIN_ERR};
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
    // const L1 : f32 = 100.0;
    // const L2 : f32 = 100.0;

    // fn get_pos(comps : &[Box<dyn Component>; 2]) -> [f32; 2] {

    //     let [ g_1, g_2 ] = comps.get_dist();
    //     [
    //         L1 * g_1.cos() + L2 * (g_1 + g_2).cos(),
    //         L1 * g_1.sin() + L2 * (g_1 + g_2).sin()
    //     ]
    // }

    // fn get_angles(pos : [f32; 2]) -> [f32; 2] {
    //     // Todo
    // }

    #[test]
    fn test_g1() {
        const U : f32 = 12.0;
        const SF : f32 = 1.5;

        let comps : [Box<dyn Component>; 2] = [ 
            Box::new(StepperCtrl::new(StepperData::mot_17he15_1504s(U, SF), PIN_ERR, PIN_ERR)),
            Box::new(StepperCtrl::new(StepperData::mot_17he15_1504s(U, SF), PIN_ERR, PIN_ERR))
        ]; 

        dbg!(comps[0].compl_times(0.1, 0.2, 0.1));
    }
// 