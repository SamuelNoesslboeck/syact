use std::sync::{Mutex, Arc};

use gpio::sysfs::*;
use serde::{Serialize, Deserialize};

use crate::{Component, LinkedData, MathActor, Delta, Gamma, Omega, Alpha, Time, Force, Inertia};
use crate::data::StepperConst;

// Use local types module
pub mod asynchr;

mod driver;
pub use driver::*;

mod meas;
pub use meas::*;

// mod paths;
// pub use paths::*;

pub mod pwm;

pub mod servo;

mod types;
pub use types::*;

/// StepperCtrl
#[derive(Debug)]
pub struct StepperCtrl
{
    /// Motor driver
    pub driver : Arc<Mutex<StepperDriver>>,
    /// Async comms
    pub comms : asynchr::AsyncStepper,

    /// Pin for controlling direction
    pub pin_dir : u16,
    /// Pin for controlling steps
    pub pin_step : u16,
    /// Pin for messuring distances
    pub pin_meas : u16, 
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct StepperCtrlDes 
{
    #[serde(serialize_with = "StepperConst::to_standard", deserialize_with = "StepperConst::from_standard")]
    pub data : StepperConst,
    pub pin_dir : u16,
    pub pin_step : u16
}

impl StepperCtrl
{   
    pub fn new(data : StepperConst, pin_dir : u16, pin_step : u16) -> Self {
        let driver = Arc::new(Mutex::new(StepperDriver::new(data, pin_dir, pin_step)));

        let ctrl = StepperCtrl { 
            pin_dir: pin_dir, 
            pin_step: pin_step, 
            pin_meas: PIN_ERR, 

            comms: asynchr::AsyncStepper::new(Arc::clone(&driver), 
                |driver_mutex , msg| { 
                    let mut driver = driver_mutex.lock().unwrap();

                    // println!("Proccessing msg in thread: {} {}", msg.0, msg.1); 
                    driver.drive(msg.0, msg.1, msg.2);

                    () 
                }),
            driver: driver
        };

        ctrl
    }

    pub fn new_sim(data : StepperConst) -> Self {
        Self::new(data, PIN_ERR, PIN_ERR)
    }

    // Movements
        pub fn step(&mut self, time : Time, ufunc : &UpdateFunc) -> StepResult {
            self.driver.lock().unwrap().step(time, ufunc)
        }

        pub fn accelerate(&mut self, stepcount : u64, omega : Omega, ufunc : &UpdateFunc) -> (StepResult, Vec<Time>) {
            self.driver.lock().unwrap().accelerate(stepcount, omega, ufunc)
        }

        pub fn drive_curve(&mut self, curve : &Vec<Time>) {
            for i in 0 .. curve.len() {
                self.step(curve[i], &UpdateFunc::None);
            }
        }

        pub fn steps(&mut self, stepcount : u64, omega : Omega, ufunc : UpdateFunc) -> StepResult {
            self.driver.lock().unwrap().steps(stepcount, omega, ufunc)
        }
    //

    // Direction
        pub fn get_dir(&self) -> bool {
            self.driver.lock().unwrap().dir
        }
        
        pub fn set_dir(&mut self, dir : bool) {
            self.driver.lock().unwrap().set_dir(dir);
        }
    //

    // Debug 
        pub fn debug_pins(&self) {
            self.driver.lock().unwrap().debug_pins();
        }
    //
}

impl SimpleMeas for StepperCtrl
{
    fn init_meas(&mut self, pin_mes : u16) {
        self.pin_meas = pin_mes;
        self.driver.lock().unwrap().set_meas(match SysFsGpioInput::open(pin_mes) {
            Ok(val) => RaspPin::Input(val),
            Err(_) => RaspPin::ErrPin
        })
    }
}

impl MathActor for StepperCtrl 
{
    fn accel_dyn(&self, omega : Omega, _ : Gamma) -> Alpha {
        self.driver.lock().unwrap().accel_dyn(omega)
    }
}

impl Component for StepperCtrl 
{
    // Data
        fn consts(&self) -> StepperConst {
            self.driver.lock().unwrap().consts.clone()
        }
    // 

    // Link
        fn link(&mut self, lk : Arc<LinkedData>) {
            self.driver.lock().unwrap().link(lk);
        }
    //

    // JSON 
        fn to_json(&self) -> Result<serde_json::Value, serde_json::Error> {
            serde_json::to_value(self)
        }
    //

    fn drive_rel(&mut self, delta : Delta, omega : Omega) -> Delta {
        self.driver.lock().unwrap().drive(delta, omega, UpdateFunc::None)
    }

    fn drive_rel_async(&mut self, delta : Delta, omega : Omega) {
        self.comms.send_msg((delta, omega, UpdateFunc::None));
    }

    fn drive_abs(&mut self, gamma : Gamma, omega : Omega) -> Delta {
        // TODO: Replace Mutex with RW Lock
        let delta = gamma - self.get_gamma();
        self.driver.lock().unwrap().drive(delta, omega, UpdateFunc::None)
    }

    fn drive_abs_async(&mut self, gamma : Gamma, omega : Omega) {
        let delta = gamma - self.get_gamma();
        self.comms.send_msg((delta, omega, UpdateFunc::None))
    }

    fn measure(&mut self, max_delta : Delta, omega : Omega, set_gamma : Gamma, accuracy : u64) -> bool {
        let mut driver = self.driver.lock().unwrap();

        driver.drive(max_delta, omega, UpdateFunc::Break(StepperDriver::__meas_helper, accuracy));
        driver.set_endpoint(set_gamma)
    }

    fn measure_async(&mut self, max_delta : Delta, omega : Omega, accuracy : u64) {
        self.comms.send_msg((max_delta, omega, UpdateFunc::Break(StepperDriver::__meas_helper, accuracy)));
    }

    fn await_inactive(&self) {
        self.comms.await_inactive();
    }

    // Position
        fn get_gamma(&self) -> Gamma {
            self.driver.lock().unwrap().get_gamma()
        }

        fn write_gamma(&mut self, pos : Gamma) {
            self.driver.lock().unwrap().write_gamma(pos);
        }

        fn get_limit_dest(&self, pos : Gamma) -> Delta {
            self.driver.lock().unwrap().get_limit_dest(pos)
        }

        fn set_endpoint(&mut self, set_gamma : Gamma) -> bool {
            self.driver.lock().unwrap().set_endpoint(set_gamma)
        }

        fn set_limit(&mut self, min : Option<Gamma>, max : Option<Gamma>) {
            self.driver.lock().unwrap().set_limit(min, max)
        }
    //

    fn apply_load_force(&mut self, force : Force) {
            self.driver.lock().unwrap().apply_load_force(force);
        }

        // Loads
        fn apply_load_inertia(&mut self, inertia : Inertia) {
            self.driver.lock().unwrap().apply_load_inertia(inertia);
        }
    //
}

impl From<StepperCtrlDes> for StepperCtrl {
    fn from(des : StepperCtrlDes) -> Self {
        StepperCtrl::new(des.data, des.pin_dir, des.pin_step)
    }
}

impl Into<StepperCtrlDes> for StepperCtrl {
    fn into(self) -> StepperCtrlDes {
        StepperCtrlDes { 
            data: self.driver.lock().unwrap().consts.clone(), 
            pin_dir: self.pin_dir, 
            pin_step: self.pin_step
        }
    }
}

// JSON_
impl Serialize for StepperCtrl {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
        where
            S: serde::Serializer {
        let raw : StepperCtrlDes = StepperCtrlDes { 
            data: self.driver.lock().unwrap().consts.clone(), 
            pin_dir: self.pin_dir, 
            pin_step: self.pin_step
        };
        raw.serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for StepperCtrl {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
        where
            D: serde::Deserializer<'de> {
        let raw = StepperCtrlDes::deserialize(deserializer)?;
        Ok(StepperCtrl::from(raw))
    }
}