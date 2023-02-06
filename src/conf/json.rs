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