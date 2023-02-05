use std::fs;
use std::rc::Rc;

use glam::Mat3;
use serde::{Serialize, Deserialize};

use crate::LinkedData;
use crate::comp::{Tool, NoTool, Cylinder, CylinderTriangle, GearBearing, PencilTool};
use super::*;

// Sub-Structs
    #[derive(Debug, Default, Clone, Copy, Serialize, Deserialize)]
    pub struct MeasInstance
    {
        pub pin : u16,
        pub set_val : f32,
        pub dist : f32
    }

    #[derive(Debug, Default, Clone, Copy, Serialize, Deserialize)]
    pub struct LimitDecl
    {
        pub max : Option<f32>,
        pub min : Option<f32>,
        pub vel : f32
    }

    #[derive(Debug, Default, Clone, Copy, Serialize, Deserialize)]
    pub struct SimData
    {
        pub mass : f32
    }

    #[derive(Debug, Default, Clone, Copy, Serialize, Deserialize)]
    pub struct AngleData
    {
        pub offset : f32,
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
            name: String::new(),
            type_name: comp.get_type_name(),

            obj: comp.to_json(),

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
            type_name: tool.get_type_name(),

            obj: tool.get_json(),

            ang: Default::default(),
            sim: Default::default(),
            meas: None,
            limit: None
        }
    }
}

#[derive(Debug)]
pub struct MachineConfig<const N : usize, const D : usize, const A : usize>
{
    pub name: String,

    pub lk : Rc<LinkedData>,

    pub anchor : Vec3,
    pub dims : [Vec3; D],
    pub axes : [Vec3; A],

    pub tools : Vec<Box<dyn Tool + Send>>,

    pub vels : [f32; N],
    pub set_vals : [f32; N],
    
    pub ang : [AngleData; N],
    pub sim : [SimData; N],
    pub meas : [MeasInstance; N],
    pub limit : [LimitDecl; N]
}

impl<const N : usize, const D : usize, const A : usize> Default for MachineConfig<N, D, A> 
{
    fn default() -> Self {
        Self {
            name: String::new(),
            lk: Default::default(),

            anchor: Default::default(),
            dims: [Default::default(); D],
            axes: [Default::default(); A], 

            tools: vec![],

            vels: [0.0; N],
            set_vals: [0.0; N],
            
            ang: [Default::default(); N],
            sim: [Default::default(); N],
            meas: [Default::default(); N],
            limit: [Default::default(); N]
        }
    }
}

impl<const N : usize, const D : usize, const A : usize> MachineConfig<N, D, A>
{
    pub fn get_axes(&self, angles : &[f32; A]) -> Vec<Mat3> {
        let mut matr = vec![];

        for i in 0 .. A {
            let axis_vec = Vec3::from(self.axes[i]).normalize();

            matr.push(
                if axis_vec == Vec3::X {
                    Mat3::from_rotation_x(angles[i])
                } else if axis_vec == Vec3::Y {
                    Mat3::from_rotation_y(angles[i])
                } else if axis_vec == Vec3::Z {
                    Mat3::from_rotation_z(angles[i])
                } else {
                    Mat3::ZERO
                }
            );
        }

        matr
    }
}

impl<const N : usize, const D : usize, const A : usize> MachineConfig<N, D, A>
{
    pub fn convert_angles(&self, angles : [f32; N], to_comp : bool) -> [f32; N] {
        let mut full_ang = [0.0; N];

        for i in 0 .. N {
            full_ang[i] = if self.ang[i].counter { 
                -angles[i] 
            } else { 
                angles[i] 
            } + if to_comp { 
                -self.ang[i].offset 
            } else {
                self.ang[i].offset
            };
        }

        full_ang
    }
}

#[derive(Serialize, Deserialize)]
pub struct JsonConfig
{
    pub name : String,

    pub lk : LinkedData,

    pub anchor : Option<[f32; 3]>,
    pub dims : Option<Vec<[f32; 3]>>,
    pub axes : Option<Vec<[f32; 3]>>,

    pub comps : Vec<ConfigElement>,
    pub tools : Vec<ConfigElement>
}

impl JsonConfig 
{
    pub fn new<const N : usize>(name : String, lk : LinkedData, anchor : Option<[f32; 3]>, dims : Option<Vec<[f32; 3]>>, axes : Option<Vec<[f32; 3]>>, 
            comps : &[Box<dyn Component>; N], tools : &Vec<Box<dyn Tool + Send>>) -> Self {
        Self { 
            name,

            lk,

            anchor,
            dims,
            axes,

            comps: create_conf_comps(comps),
            tools: create_conf_tools(tools)
        }
    }

    pub fn get_machine<const N : usize, const D : usize, const A : usize>(&self) -> Result<(MachineConfig<N, D, A>, [Box<dyn Component>; N]), std::io::Error> {
        if self.comps.len() != N {
            return Err(std::io::Error::new(std::io::ErrorKind::InvalidData, 
                format!("Not enough components for machine! [Required: {}, Given: {}]", N, self.comps.len())))
        }

        let mut comps = vec![];
        let mut mach : MachineConfig<N, D, A> = Default::default();
        
        // Init
        mach.name = self.name.clone();
        mach.lk = Rc::new(self.lk.clone());

        mach.anchor = match self.anchor {
            Some(anchor_raw) => Vec3::from(anchor_raw),
            None => Default::default()
        };

        mach.dims = match &self.dims {
            Some(dims) => match dims.iter().map(|axis_raw| Vec3::from(*axis_raw)).collect::<Vec<Vec3>>().try_into() {
                Ok(val) => val, 
                Err(_) => return Err(std::io::Error::new(
                    std::io::ErrorKind::InvalidData, format!("Not enough dimensions defined for machine! [Required: {}, Given: {}]", D, dims.len())))
            },
            None => return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidData, format!("Not enough dimensions defined for machine! [Required: {}, Given: 0]", D)))
        };

        mach.axes = match &self.axes {
            Some(axes) => match axes.iter().map(|axis_raw| Vec3::from(*axis_raw)).collect::<Vec<Vec3>>().try_into() {
                Ok(val) => val, 
                Err(_) => return Err(std::io::Error::new(
                    std::io::ErrorKind::InvalidData, format!("Not enough axes defined for machine! [Required: {}, Given: {}]", A, axes.len())))
            },
            None => return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidData, format!("Not enough axes defined for machine! [Required: {}, Given: 0]", A)))
        };

        mach.tools = self.tools.iter().map(
            |tool_raw| tool_raw.get_tool().unwrap()
        ).collect();
        
        for i in 0 .. N {
            let mut comp = self.comps[i].get_comp().unwrap();

            mach.ang[i] = self.comps[i].ang;
            mach.sim[i] = self.comps[i].sim;

            // Collected arrays
            match self.comps[i].limit {
                Some(lim) => { 
                    

                    mach.vels[i] = lim.vel;
                    mach.limit[i] = lim;
                }, 
                None => { }
            };

            match self.comps[i].meas {
                Some(meas) => {
                    comp.init_meas(meas.pin);

                    mach.set_vals[i] = meas.set_val;
                    mach.meas[i] = meas;
                },
                None => { }
            }; 

            comps.push(comp);
        }

        Ok((mach, comps.try_into().unwrap()))
    }

    pub fn to_string_pretty(&self) -> String {
        serde_json::to_string_pretty(self).unwrap()
    }

    // File I/O
        pub fn save_to_file(&self, path : &str) {
            fs::write(path, self.to_string_pretty()).unwrap()
        }

        pub fn read_from_file(path : &str) -> Self {
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