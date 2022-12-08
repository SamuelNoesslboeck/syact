use std::collections::HashMap;

pub type GCodeFunc<T, E> = fn (&mut T, &GCode, &Args) -> E;

pub type Letter = gcode::Mnemonic;
pub type GCode = gcode::GCode;
pub type Args = [gcode::Word];

pub type NumEntries<T, E> = HashMap<u32, GCodeFunc<T, E>>;
pub type LetterEntries<T, E> = HashMap<Letter, NumEntries<T, E>>;

pub struct Interpreter<T, E>
{
    pub funcs : LetterEntries<T, E>,
    pub mach : T
}

pub struct Path
{

}

impl<T, E> Interpreter<T, E>
{   
    pub fn new(mach : T, funcs : LetterEntries<T, E>) -> Self {
        return Interpreter {
            funcs,
            mach
        }
    }

    pub fn log_ln(&self, line : &str) {
        println!("{}", line);
    }

    pub fn interpret(&mut self, gc_str : &str) -> Vec<E> {
        let mut line_count : u64 = 0;
        let mut res = vec![];

        for gc_line in gcode::parse(gc_str) {
            let func_res = get_func(&self.funcs, &gc_line);

            res.push(func_res.and_then(|func| {
                Some(func(&mut self.mach, &gc_line, gc_line.arguments()))
            }).unwrap());

            line_count += 1;
        }

        self.log_ln(format!("Interpreted {} lines", line_count).as_str());

        res
    }
}


/// Get the GCode Function stored for the given code
pub fn get_func<'a, T, E>(funcs : &'a LetterEntries<T, E>, gc : &'a GCode) -> Option<&'a GCodeFunc<T, E>> {
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