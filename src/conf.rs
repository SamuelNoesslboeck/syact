use serde::{Serialize, Deserialize};

use super::*;

#[derive(Serialize, Deserialize)]
pub struct ConfigElement 
{
    pub name : String,
    pub obj : serde_json::Value
}

impl ConfigElement 
{
    pub fn new(comp : &Box<dyn Component>) -> Self {
        ConfigElement {
            name: comp.get_name(),
            obj: comp.to_json()
        }
    }
}

#[derive(Serialize, Deserialize)]
pub struct Configuration<const N : usize>
{

}

pub fn create_conf_elems<const N : usize>(comps : &[Box<dyn Component>; N]) -> serde_json::Value {
    let mut values = vec![];

    for i in 0 .. N {
        values.push(
            serde_json::to_value(
                ConfigElement::new(&comps[i])
            ).unwrap()
        );
    }

    serde_json::Value::Array(values)
}

// pub fn read_conf<const N : usize>(comf : &serde_json::Value) -> &[Box<dyn Component>; N] {

// }