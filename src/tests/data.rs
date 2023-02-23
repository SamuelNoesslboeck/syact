use std::collections::HashMap;

use serde::{Serialize, Deserialize};

use crate::{StepperCtrl, Component, StepperConst};
use crate::ctrl::PIN_ERR;
use crate::gcode::{GCodeFunc, GCode, Args};

mod stepper_data 
{
    use super::*;

    #[derive(Debug, Serialize, Deserialize)]
    struct Test {
        #[serde(serialize_with = "StepperConst::to_standard", deserialize_with = "StepperConst::from_standard")]
        data: StepperConst
    }

    #[test]
    fn json_io() {
        let json_init = serde_json::json!(Test { data: StepperConst::MOT_17HE15_1504S });
        let data : Test = serde_json::from_value(json_init.clone()).unwrap();

        // Check if data is valid
        assert_eq!(data.data, StepperConst::MOT_17HE15_1504S);
        
        // Check if Json is valid
        assert_eq!(serde_json::to_value(&data).unwrap(), json_init);
    }

    #[test]
    fn conf_io() {
        let comps : [Box<dyn Component>; 2] = [ 
            Box::new(
                crate::comp::CylinderTriangle::new(
                    crate::comp::Cylinder::new(
                        StepperCtrl::new(StepperConst::MOT_17HE15_1504S, PIN_ERR, PIN_ERR), 
                    1.5),
                100.0, 200.0)),
            Box::new(
                crate::comp::GearBearing {
                    ctrl: StepperCtrl::new(StepperConst::MOT_17HE15_1504S, PIN_ERR, PIN_ERR),
                    ratio: 1.5
                })
        ]; 

        println!("{}", serde_json::to_string_pretty(
            &crate::conf::create_conf_comps(&comps)
        ).unwrap());

        let tools : [Box<dyn crate::Tool + Send>; 2] = [
            Box::new(
                crate::comp::NoTool::new()
            ),
            Box::new(
                crate::comp::PencilTool::new(100.0, 0.25)
            )
        ]; 

        println!("{}", serde_json::to_string_pretty(
            &crate::conf::create_conf_tools(&Vec::from(tools))
        ).unwrap());
    }   
}


mod gcode
{
    use super::*; 

    struct Data 
    {
        pub pos : f32
    }

    fn g_0(data : &mut Data, _gc : &GCode, _args : &Args) -> Option<f32> {
        data.pos += 10.0;
        Some(data.pos)
    }

    fn g_1(data : &mut Data, _gc : &GCode, _args : &Args) -> Option<f32> {
        data.pos -= 5.0;
        Some(data.pos)
    }

    #[test]
    fn test_gcode() {
        let map = HashMap::from([
            ( ::gcode::Mnemonic::General, HashMap::from([
                ( 0u32, g_0 as GCodeFunc<Data, Option<f32>> ),
                ( 1u32, g_1 as GCodeFunc<Data, Option<f32>> )
            ]) )
        ]);

        let mut intpr = crate::gcode::Interpreter::new(Data { pos: 0.0 }, map);

        let res = intpr.interpret("G0\nG1", |_| { Some(0.0) });
                
        assert_eq!(res, vec![Some(10.0), Some(5.0)]);
    }
}