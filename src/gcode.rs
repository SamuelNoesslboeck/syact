use std::collections::HashMap;
use std::fs;

pub type GCodeFunc<T, E> = fn (&mut T, &GCode, &Args) -> E;
pub type ToolChangeFunc<T, E> = fn (&mut T, usize) -> E;

pub type Letter = gcode::Mnemonic;
pub type GCode = gcode::GCode;
pub type Args = [gcode::Word];

pub type NumEntries<T, R> = HashMap<u32, GCodeFunc<T, R>>;
pub type LetterEntries<T, R> = HashMap<Letter, NumEntries<T, R>>;

pub type NotFoundFunc<R> = fn (GCode) -> R;

pub struct Interpreter<T, R>
{
    pub funcs : LetterEntries<T, R>,
    pub tool_change : Option<ToolChangeFunc<T, R>>,

    pub mach : T
}

impl<T, R> Interpreter<T, R>
{   
    pub fn new(mach : T, tool_change : Option<ToolChangeFunc<T, R>>, funcs : LetterEntries<T, R>) -> Self {
        return Interpreter {
            funcs,
            tool_change,

            mach
        }
    }
    
    pub fn interpret(&mut self, gc_str : &str, not_found : NotFoundFunc<R>) -> Vec<R> {
        let mut res = vec![]; 

        for gc_str_line in gc_str.split('\n') {
            for gc_line in gcode::parse(gc_str_line) {
                if gc_line.mnemonic() == Letter::ToolChange {
                    res.push(match self.tool_change {
                        Some(func) => func(&mut self.mach, gc_line.major_number() as usize),
                        None => not_found(gc_line)
                    });

                    continue;
                }

                let func_res = get_func(&self.funcs, &gc_line);

                res.push(match func_res {
                    Some(func) => func(&mut self.mach, &gc_line, gc_line.arguments()),
                    None => not_found(gc_line)
                });
            }
        }

        res
    }

    pub fn interpret_file(&mut self, file_path : &str, not_found : NotFoundFunc<R>) -> Vec<R> {
        self.interpret(
            fs::read_to_string(file_path).unwrap().as_str(), not_found)
    }
}


/// Get the GCode Function stored for the given code
pub fn get_func<'a, T, E>(funcs : &'a LetterEntries<T, E>, gc : &'a GCode) -> Option<&'a GCodeFunc<T, E>> {
    funcs.get(&gc.mnemonic()).and_then(|v| {
        v.get(&gc.major_number())
    })
}

// Argument parsing
pub fn arg_by_letter(args : &Args, letter : char) -> Option<f32> {
    for i in 0 .. args.len() {
        if args[i].letter == letter {
            return Some(args[i].value);
        }
    }

    None
}

pub fn args_by_letter(args : &Args, letter : char) -> Vec<f32> {
    let mut letters = Vec::new();

    for i in 0 .. args.len() {
        if args[i].letter == letter {
            letters.push(args[i].value);
        }
    }

    letters
}

pub fn args_by_letter_fixed<const N : usize>(args : &Args, letter : char) -> [Option<f32>; N] {
    let mut letters = [None; N]; 
    let mut l_index : usize = 0;

    for i in 0 .. args.len() {
        if args[i].letter == letter {
            letters[l_index] = Some(args[i].value);
            l_index += 1;
        }
    }   

    letters
}

pub fn args_by_iterate_fixed<const N : usize>(args : &Args, base_letter : char) -> [Option<f32>; N] {
    let mut letters = [None; N];
    
    for i in 0 .. N {
        letters[i] = arg_by_letter(args, (base_letter as u8 + i as u8) as char);
    }

    letters
}