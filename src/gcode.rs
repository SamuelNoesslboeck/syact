use std::collections::HashMap;

use gcode::{Mnemonic};

pub type GCodeFunc<T> = fn (&Interpreter<T>, &GCode, &T) -> Option<()>;

pub type Letter = Mnemonic;
pub type GCode = gcode::GCode;

pub type NumEntries<T> = HashMap<u32, GCodeFunc<T>>;
pub type LetterEntries<T> = HashMap<Letter, NumEntries<T>>;

pub struct Interpreter<'a, T>
{
    funcs : LetterEntries<T>,
    pub mach : &'a T
}

impl<'a, T> Interpreter<'a, T>
{   
    pub fn new(mach : &'a T, funcs : LetterEntries<T>) -> Self {
        return Interpreter {
            funcs,
            mach
        }
    }

    pub fn log_ln(&self, line : &str) {
        println!("{}", line);
    }

    pub fn interpret(&self, gc_str : &str) {
        let mut line_count : u64 = 0;

        for gc_line in gcode::parse(gc_str) {
            self.get_func(&gc_line).and_then(|func| {
                func(&self, &gc_line, self.mach)
            });

            line_count += 1;
        }

        self.log_ln(format!("Interpreted {} lines", line_count).as_str());
    }
    
    /// Get the GCode Function stored for the given code
    pub fn get_func(&self, gc : &GCode) -> Option<&GCodeFunc<T>> {
        self.funcs.get(&gc.mnemonic()).and_then(|v| {
            v.get(&gc.major_number())
        })
    }
}