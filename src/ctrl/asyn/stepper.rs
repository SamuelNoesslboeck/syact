extern crate alloc;
use alloc::sync::Arc;

use std::sync::Mutex;

use crate::comp::asyn::AsyncComp;
use crate::{Component, StepperConst};
use crate::ctrl::asyn::AsyncHandler;
use crate::units::*;

// Steppers
type StepperMsg = CompFunc;
type StepperRes = ();
type AsyncStepperHandler = AsyncHandler<StepperMsg, StepperRes>;
//

#[derive(Debug)]
enum CompFunc {
    DriveRel(Delta, Omega),
    DriveAbs(Gamma, Omega), 
    Measure(Delta, Omega, Gamma, u64)
}

#[derive(Debug)]
pub struct AsyncCtrl {
    comp : Arc<Mutex<Box<dyn Component + Send>>>,
    handler : AsyncStepperHandler
}

impl AsyncCtrl {
    pub fn new(comp : Arc<Mutex<Box<dyn Component + Send>>>) -> Self {
        let handler = AsyncStepperHandler::new(comp.clone(),
            |driver_mutex , msg| { 
                let mut driver = driver_mutex.lock().unwrap();

                match msg {
                    CompFunc::DriveRel(delta, omega) => driver.drive_rel(delta, omega),
                    CompFunc::DriveAbs(gamma, omega) => driver.drive_abs(gamma, omega),
                    CompFunc::Measure(delta, omega, set_gamma, acc) => { driver.measure(delta, omega, set_gamma, acc); Delta::ZERO }
                };

                () 
            }
        );

        AsyncCtrl { 
            comp: comp, 
            handler: handler
        }
    }
}

impl crate::meas::SimpleMeas for AsyncCtrl {
    fn init_meas(&mut self, pin_meas : u16) {
        self.comp.lock().unwrap().init_meas(pin_meas);
    }
}

impl crate::math::MathActor for AsyncCtrl {
    fn accel_dyn(&self, omega : Omega, gamma : Gamma) -> Alpha {
        self.comp.lock().unwrap().accel_dyn(omega, gamma)
    }
}

impl Component for AsyncCtrl {
    fn consts(&self) -> StepperConst {
        self.comp.lock().unwrap().consts()
    }

    fn to_json(&self) -> Result<serde_json::Value, serde_json::Error> {
        self.comp.lock().unwrap().to_json()
    }

    fn link(&mut self, lk : crate::data::LinkedData) {
        if let Ok(mut s_comp) = self.comp.lock() {
            s_comp.link(lk);
        }
    }

    fn drive_rel(&mut self, mut delta : Delta, mut omega : Omega) -> Delta {
        let gamma = self.get_gamma(); 

        delta = self.delta_for_super(delta, gamma);
        omega = self.omega_for_super(omega, gamma);

        let res = if let Ok(mut s_comp) = self.comp.lock() {
            s_comp.drive_rel(delta, omega)
        } else { Delta::ZERO }; 
        
        self.delta_for_this(res, self.gamma_for_super(gamma))
    }

    fn drive_abs(&mut self, mut gamma : Gamma, mut omega : Omega) -> Delta {
        omega = self.omega_for_super(omega, gamma);
        gamma = self.gamma_for_super(gamma);

        let res = if let Ok(mut s_comp) = self.comp.lock() {
            s_comp.drive_abs(gamma, omega)
        } else { Delta::ZERO }; 

        self.delta_for_this(res, gamma)
    }

    fn measure(&mut self, mut delta : Delta, mut omega : Omega, mut set_gamma : Gamma, accuracy : u64) -> bool {
        delta = self.delta_for_super(delta, self.get_gamma());
        omega = self.omega_for_super(omega, self.get_gamma());
        set_gamma = self.gamma_for_super(set_gamma);

        if let Ok(mut s_comp) = self.comp.lock() {
            s_comp.measure(delta, omega, set_gamma, accuracy)
        } else { false }
    }

    fn get_gamma(&self) -> Gamma {
        let super_len = if let Ok(s_comp) = self.comp.lock() {
            s_comp.get_gamma()
        } else { Gamma::ZERO };

        self.gamma_for_this(super_len)
    }

    fn write_gamma(&mut self, mut gamma : Gamma) {
        gamma = self.gamma_for_super(gamma);

        if let Ok(mut s_comp) = self.comp.lock() {
            s_comp.write_gamma(gamma);
        }
    }

    fn get_limit_dest(&self, mut gamma : Gamma) -> Delta {
        gamma = self.gamma_for_super(gamma);

        let delta = if let Ok(s_comp) = self.comp.lock() {
            s_comp.get_limit_dest(gamma)
        } else { Delta::ZERO };

        self.delta_for_this(delta, gamma)
    }

    fn set_endpoint(&mut self, mut set_gamma : Gamma) -> bool {
        set_gamma = self.gamma_for_super(set_gamma);

        if let Ok(mut s_comp) = self.comp.lock() {
            s_comp.set_endpoint(set_gamma)
        } else { false }
    }

    fn set_limit(&mut self, mut min : Option<Gamma>, mut max : Option<Gamma>) {
        min = match min {
            Some(min) => Some(self.gamma_for_super(min)),
            None => None
        }; 

        max = match max {
            Some(max) => Some(self.gamma_for_super(max)),
            None => None
        };

        if let Ok(mut s_comp) = self.comp.lock() {
            s_comp.set_limit(min, max)
        }
    }

    fn apply_load_force(&mut self, mut force : Force) { // TODO: Add overload protection
        force = Force(self.gamma_for_this(Gamma(force.0)).0);

        if let Ok(mut s_comp) = self.comp.lock() {
            s_comp.apply_load_force(force);
        }
    }

    fn apply_load_inertia(&mut self, mut inertia : Inertia) {
        inertia = Inertia(self.gamma_for_this(self.gamma_for_this(Gamma(inertia.0))).0);

        if let Ok(mut s_comp) = self.comp.lock() {
            s_comp.apply_load_inertia(inertia);
        }
    }
}

impl AsyncComp for AsyncCtrl {
    fn drive_rel_async(&mut self, delta : Delta, omega : Omega) {
        self.handler.send_msg(CompFunc::DriveRel(delta, omega))
    }

    fn drive_abs_async(&mut self, gamma : Gamma, omega : Omega) {
        self.handler.send_msg(CompFunc::DriveAbs(gamma, omega))
    }

    fn measure_async(&mut self, delta : Delta, omega : Omega, set_gamma : Gamma, accuracy : u64) {
        self.handler.send_msg(CompFunc::Measure(delta, omega, set_gamma, accuracy))
    }

    fn await_inactive(&self) {
        self.handler.await_inactive()
    }
}