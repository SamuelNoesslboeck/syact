use std::collections::HashMap;

use gcode::{Mnemonic, GCode};

pub type GCodeFunc = fn (&Interpreter, &GCode) -> Option<()>;

pub struct Interpreter 
{
    funcs : HashMap<Mnemonic, HashMap<u32, GCodeFunc>>
}

impl Interpreter
{   
    pub fn new(funcs : HashMap<Mnemonic, HashMap<u32, GCodeFunc>>) -> Self {
        return Interpreter {
            funcs 
        }
    }

    pub fn log_ln(&self, line : &str) {
        println!("{}", line);
    }

    pub fn interpret(&self, gc_str : &str) {
        let mut line_count : u64 = 0;

        for gc_line in gcode::parse(gc_str) {
            self.get_func(&gc_line).and_then(|func| {
                func(&self, &gc_line)
            });

            line_count += 1;
        }

        self.log_ln(format!("Interpreted {} lines", line_count).as_str());
    }
    
    /// Get the GCode Function stored for the given code
    pub fn get_func(&self, gc : &GCode) -> Option<&GCodeFunc> {
        self.funcs.get(&gc.mnemonic()).and_then(|v| {
            v.get(&gc.major_number())
        })
    }
}