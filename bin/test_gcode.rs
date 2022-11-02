use std::collections::HashMap;

use gcode::{Mnemonic, GCode};
use stepper_lib::gcode::{Interpreter, GCodeFunc};

fn g_0(intpr : &Interpreter, _gc : &GCode) -> Option<()> {
    intpr.log_ln("G0 function executed");
    None
}

fn g_1(intpr : &Interpreter, _gc : &GCode) -> Option<()> {
    intpr.log_ln("G1 function executed");
    None
}

fn main() {
    let map = HashMap::from([
        ( Mnemonic::General, HashMap::from([
            ( 0u32, g_0 as GCodeFunc ),
            ( 1u32, g_1 as GCodeFunc )
        ]) )
    ]);

    let intpr = Interpreter::new(map);

    intpr.interpret("G0\nG1");
}