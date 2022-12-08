use std::collections::HashMap;

use gcode::{Mnemonic, GCode};
use stepper_lib::gcode::{Interpreter, GCodeFunc, Args};

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

fn main() {
    let map = HashMap::from([
        ( Mnemonic::General, HashMap::from([
            ( 0u32, g_0 as GCodeFunc<Data, Option<()>> ),
            ( 1u32, g_1 as GCodeFunc<Data, Option<()>> )
        ]) )
    ]);

    let mut intpr = Interpreter::new(Data { pos: 0.0 }, map);

    intpr.interpret("G0\nG1");
}