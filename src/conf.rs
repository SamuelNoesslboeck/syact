use std::fs;

use glam::Mat3;
use serde::{Serialize, Deserialize};

use crate::LinkedData;
use crate::comp::{Tool, NoTool, Cylinder, CylinderTriangle, GearBearing, PencilTool};
use super::*;

// Sub-Structs
    #[derive(Debug, Clone, Serialize, Deserialize)]
    pub struct MeasInstance
    {
        pub pin : u16,
        pub set_val : f32,
        pub dist : u16
    }

    #[derive(Debug, Clone, Serialize, Deserialize)]
    pub struct LimitDecl
    {
        pub max : Option<f32>,
        pub min : Option<f32>,
        pub omega_max : Option<f32>
    }
//

#[derive(Serialize, Deserialize)]
pub struct ConfigElement 
{
    pub name: Option<String>, 
    pub type_name : String,

    pub obj : serde_json::Value,

    pub mass : Option<f32>,
    pub meas : Option<MeasInstance>,
    pub limit : Option<LimitDecl>
}

impl ConfigElement 
{
    pub fn get_comp(&self) -> Option<Box<dyn Component>> {
        match self.type_name.as_str() {
            "stepper_lib::ctrl::StepperCtrl" => Some(Box::new(
                    serde_json::from_value::<StepperCtrl>(self.obj.clone()).unwrap()
            )),
            "stepper_lib::comp::cylinder::Cylinder" => Some(Box::new(
                serde_json::from_value::<Cylinder>(self.obj.clone()).unwrap()
            )),
            "stepper_lib::comp::cylinder_triangle::CylinderTriangle" => Some(Box::new(
                serde_json::from_value::<CylinderTriangle>(self.obj.clone()).unwrap()
            )),
            "stepper_lib::comp::gear_bearing::GearBearing" => Some(Box::new(
                serde_json::from_value::<GearBearing>(self.obj.clone()).unwrap()
            )), 
            _ => None
        }
    }

    pub fn get_tool(&self) -> Option<Box<dyn Tool + Send>> {
        match self.type_name.as_str() {
            "stepper_lib::comp::tool::NoTool" => Some(Box::new(
                NoTool::new()
            )),
            "stepper_lib::comp::tool::PencilTool" => Some(Box::new(
                serde_json::from_value::<PencilTool>(self.obj.clone()).unwrap()
            )),
            _ => None
        }
    }
}

impl From<&Box<dyn Component>> for ConfigElement 
{
    fn from(comp: &Box<dyn Component>) -> Self {
        Self {
            name: None,
            type_name: comp.get_type_name(),

            obj: comp.to_json(),

            meas: None,
            limit: None
        }
    }
}

impl From<&Box<dyn Tool + Send>> for ConfigElement 
{
    fn from(tool: &Box<dyn Tool + Send>) -> Self {
        Self {
            name: None,
            type_name: tool.get_type_name(),

            obj: tool.get_json(),

            meas: None,
            limit: None
        }
    }
}

#[derive(Serialize, Deserialize)]
pub struct JsonConfig
{
    pub name : String,

    pub lk : LinkedData,

    pub anchor : Option<[f32; 3]>,
    pub dim : Option<Vec<[f32; 3]>>,
    pub axes : Option<Vec<[f32; 3]>>,

    pub comps : Vec<ConfigElement>,
    pub tools : Vec<ConfigElement>
}

impl JsonConfig 
{
    pub fn new<const N : usize>(name : String, lk : LinkedData, anchor : Option<[f32; 3]>, dim : Option<Vec<[f32; 3]>>, axes : Option<Vec<[f32; 3]>>, 
            comps : &[Box<dyn Component>; N], tools : &Vec<Box<dyn Tool + Send>>) -> Self {
        Self { 
            name,

            lk,

            anchor,
            dim,
            axes,

            comps: create_conf_comps(comps),
            tools: create_conf_tools(tools)
        }
    }

    pub fn get_comps<const N : usize>(&self) -> [Box<dyn Component>; N] {
        let mut comps = vec![];
        for i in 0 .. N {
            let mut comp = self.comps[i].get_comp().unwrap(); 
            
            if let Some(meas) = &self.comps[i].meas {
                comp.init_meas(meas.pin);
            }

            comps.push(comp);
        }
        comps.try_into().unwrap_or_else(
            |v: Vec<Box<dyn Component>>| panic!("Wrong number of components in configuration! (Required: {}, Found: {})", N, v.len()))
    }

    pub fn get_tools<const N : usize>(&self) -> Vec<Box<dyn Tool + Send>> {
        let mut tools = vec![];
        for i in 0 .. N {
            let tool = self.comps[i].get_tool().unwrap();
            tools.push(tool);
        }
        tools
    }

    pub fn get_axes<const N : usize>(&self, angles : &[f32; N]) -> Vec<Mat3> {
        let mut matr = vec![];

        if let Some(axes) = &self.axes {
            for i in 0 .. axes.len() {
                let axis_vec = Vec3::from(axes[i]).normalize();

                matr.push(
                    if axis_vec == Vec3::X {
                        Mat3::from_rotation_x(angles[i])
                    } else if axis_vec == Vec3::Y {
                        Mat3::from_rotation_y(angles[i])
                    } else if axis_vec == Vec3::Z {
                        Mat3::from_rotation_z(angles[i])
                    } else if axis_vec == Vec3::NEG_X {
                        Mat3::from_rotation_x(-angles[i])
                    } else if axis_vec == Vec3::NEG_Y {
                        Mat3::from_rotation_y(-angles[i])
                    } else if axis_vec == Vec3::NEG_Z {
                        Mat3::from_rotation_z(-angles[i])
                    } else {
                        Mat3::ZERO
                    }
                );
            }
        }

        matr
    }

    pub fn get_dim(&self) -> Vec<Vec3> {
        let mut dims = vec![];

        if let Some(dims_raw) = &self.dim {
            dims = dims_raw.iter().map(|dim_raw| { Vec3::from(*dim_raw) }).collect();
        }

        dims
    }

    pub fn to_string_pretty(&self) -> String {
        serde_json::to_string_pretty(self).unwrap()
    }

    // File I/O
        pub fn save_to_file(&self, path : &str) {
            fs::write(path, self.to_string_pretty()).unwrap()
        }

        pub fn read_from_file(&self, path : &str) -> Self {
            serde_json::from_str(fs::read_to_string(path).unwrap().as_str()).unwrap()
        }
    // 
}

pub fn create_conf_comps<const N : usize>(comps : &[Box<dyn Component>; N]) -> Vec<ConfigElement> {
    let mut values = vec![];
    for i in 0 .. N {
        values.push(
            ConfigElement::from(&comps[i])
        );
    }
    values
}

pub fn create_conf_tools(tools : &Vec<Box<dyn Tool + Send>>) -> Vec<ConfigElement> {
    let mut values = vec![];
    for tool in tools {
        values.push(
            ConfigElement::from(
                tool
            )
        );
    }
    values
}

// pub fn read_conf<const N : usize>(comf : &serde_json::Value) -> &[Box<dyn Component>; N] {

// }