use std::collections::HashMap;

pub type GCodeFunc<T> = fn (&mut T, &GCode, &Args) -> Option<()>;

pub type Letter = gcode::Mnemonic;
pub type GCode = gcode::GCode;
pub type Args = [gcode::Word];

pub type NumEntries<T> = HashMap<u32, GCodeFunc<T>>;
pub type LetterEntries<T> = HashMap<Letter, NumEntries<T>>;

pub struct Interpreter<T>
{
    pub funcs : LetterEntries<T>,
    pub mach : T
}

pub struct Path
{

}

impl<T> Interpreter<T>
{   
    pub fn new(mach : T, funcs : LetterEntries<T>) -> Self {
        return Interpreter {
            funcs,
            mach
        }
    }

    pub fn log_ln(&self, line : &str) {
        println!("{}", line);
    }

    pub fn interpret(&mut self, gc_str : &str) {
        let mut line_count : u64 = 0;

        for gc_line in gcode::parse(gc_str) {
            let func_res = get_func(&self.funcs, &gc_line);

            func_res.and_then(|func| {
                func(&mut self.mach, &gc_line, gc_line.arguments())
            });

            line_count += 1;
        }

        self.log_ln(format!("Interpreted {} lines", line_count).as_str());
    }
}


/// Get the GCode Function stored for the given code
pub fn get_func<'a, T>(funcs : &'a LetterEntries<T>, gc : &'a GCode) -> Option<&'a GCodeFunc<T>> {
    funcs.get(&gc.mnemonic()).and_then(|v| {
        v.get(&gc.major_number())
    })
}

pub fn get_arg_letter(args : &Args, letter : char) -> Option<f32> {
    for i in 0 .. args.len() {
        if args[i].letter == letter {
            return Some(args[i].value);
        }
    }

    None
}