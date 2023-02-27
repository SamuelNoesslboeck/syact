#![doc = r"File"]
//! 

extern crate alloc;
use alloc::string::String;
use alloc::boxed::Box;

use serde::{Serialize, Deserialize};

use crate::{Component, comp::Tool, Gamma, Delta, Omega};

// Sub-Structs
    #[derive(Debug, Default, Clone, Copy, Serialize, Deserialize)]
    pub struct MeasInstance
    {
        pub pin : u16,
        pub set_val : Gamma,
        pub dist : Delta
    }

    #[derive(Debug, Default, Clone, Copy, Serialize, Deserialize)]
    pub struct LimitDecl
    {
        pub max : Option<Gamma>,
        pub min : Option<Gamma>,
        pub vel : Omega
    }

    #[derive(Debug, Default, Clone, Copy, Serialize, Deserialize)]
    pub struct SimData
    {
        pub mass : f32
    }

    #[derive(Debug, Default, Clone, Copy, Serialize, Deserialize)]
    pub struct AngleData
    {
        #[serde(default)]
        pub offset : Delta,
        #[serde(default)]
        pub counter : bool
    }
//

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConfigElement 
{
    #[serde(default)]
    pub name: String, 
    pub type_name : String,

    pub obj : serde_json::Value,

    #[serde(default)]
    pub ang : AngleData,
    #[serde(default)]
    pub sim : SimData,

    pub meas : Option<MeasInstance>,
    pub limit : Option<LimitDecl>
}

impl ConfigElement 
{
    pub fn get_comp(&self) -> Option<Box<dyn Component>> {
        match self.type_name.as_str() {
            "stepper_lib::ctrl::StepperCtrl" => Some(Box::new(
                    serde_json::from_value::<crate::StepperCtrl>(self.obj.clone()).unwrap()
            )),
            "stepper_lib::comp::cylinder::Cylinder" => Some(Box::new(
                serde_json::from_value::<crate::comp::Cylinder>(self.obj.clone()).unwrap()
            )),
            "stepper_lib::comp::cylinder_triangle::CylinderTriangle" => Some(Box::new(
                serde_json::from_value::<crate::comp::CylinderTriangle>(self.obj.clone()).unwrap()
            )),
            "stepper_lib::comp::gear_bearing::GearBearing" => Some(Box::new(
                serde_json::from_value::<crate::comp::GearBearing>(self.obj.clone()).unwrap()
            )), 
            _ => None
        }
    }

    pub fn get_tool(&self) -> Result<Box<dyn Tool + Send>, std::io::Error> {
        match self.type_name.as_str() {
            "stepper_lib::comp::tool::NoTool" => Ok(Box::new(
                crate::comp::NoTool::new()
            )),
            "stepper_lib::comp::tool::PencilTool" => Ok(Box::new(
                serde_json::from_value::<crate::comp::PencilTool>(self.obj.clone()).unwrap()
            )),
            _ => Err(
                std::io::Error::new(std::io::ErrorKind::InvalidData, format!("The type name {:?} does not match any known type to the software", self.type_name))
            )
        }
    }
}

impl From<&Box<dyn Component>> for ConfigElement 
{
    fn from(comp: &Box<dyn Component>) -> Self {
        Self {
            name: String::new(),
            type_name: String::from(comp.get_type_name()),

            obj: comp.to_json().unwrap(),

            ang: Default::default(),
            sim: Default::default(),
            meas: None,
            limit: None
        }
    }
}

impl From<&Box<dyn Tool + Send>> for ConfigElement 
{
    fn from(tool: &Box<dyn Tool + Send>) -> Self {
        Self {
            name: String::new(),
            type_name: String::from(tool.get_type_name()),

            obj: tool.get_json(),

            ang: Default::default(),
            sim: Default::default(),
            meas: None,
            limit: None
        }
    }
}