use std::collections::HashMap;

use gcode::{Mnemonic};

pub type GCodeFunc = fn (&Interpreter, &GCode) -> Option<()>;

pub type Letter = Mnemonic;
pub type GCode = gcode::GCode;

pub type NumEntries = HashMap<u32, GCodeFunc>;
pub type LetterEntries = HashMap<Letter, NumEntries>;

pub struct Interpreter 
{
    funcs : LetterEntries
}

impl Interpreter
{   
    pub fn new(funcs : LetterEntries) -> Self {
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