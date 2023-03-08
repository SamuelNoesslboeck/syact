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
    fn init_meas(&mut self, pin_meas : u8) {
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

    // Conversions
        #[inline(always)]
        fn gamma_for_super(&self, this_gamma : Gamma) -> Gamma {
            this_gamma
        }

        #[inline(always)]
        fn gamma_for_this(&self, super_gamma : Gamma) -> Gamma {
            super_gamma
        }

        #[inline(always)]
        fn delta_for_super(&self, this_delta : Delta, _ : Gamma) -> Delta {
            this_delta
        }

        #[inline(always)]
        fn delta_for_this(&self, super_delta : Delta, _ : Gamma) -> Delta {
            super_delta
        }

        #[inline(always)]
        fn omega_for_super(&self, this_omega : Omega, _ : Gamma) -> Omega {
            this_omega
        }

        #[inline(always)]
        fn omega_for_this(&self, super_omega : Omega, _ : Gamma) -> Omega {
            super_omega
        }

        #[inline(always)]
        fn alpha_for_super(&self, this_alpha : Alpha, _ : Gamma) -> Alpha {
            this_alpha
        }

        #[inline(always)]
        fn alpha_for_this(&self, super_alpha : Alpha, _ : Gamma) -> Alpha {
            super_alpha
        }
    //

    fn link(&mut self, lk : crate::data::LinkedData) {
        if let Ok(mut s_comp) = self.comp.lock() {
            s_comp.link(lk);
        }
    }

    fn drive_rel(&mut self, delta : Delta, omega : Omega) -> Delta {
        if let Ok(mut s_comp) = self.comp.lock() {
            s_comp.drive_rel(delta, omega)
        } else { Delta::ZERO }
    }

    fn drive_abs(&mut self, gamma : Gamma, omega : Omega) -> Delta {
        if let Ok(mut s_comp) = self.comp.lock() {
            s_comp.drive_abs(gamma, omega)
        } else { Delta::ZERO }
    }

    fn measure(&mut self, delta : Delta, omega : Omega, set_gamma : Gamma, accuracy : u64) -> bool {
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

    fn write_gamma(&mut self, gamma : Gamma) {
        if let Ok(mut s_comp) = self.comp.lock() {
            s_comp.write_gamma(gamma);
        }
    }

    fn get_limit_dest(&self, gamma : Gamma) -> Delta {
        if let Ok(s_comp) = self.comp.lock() {
            s_comp.get_limit_dest(gamma)
        } else { Delta::ZERO }
    }

    fn set_endpoint(&mut self, set_gamma : Gamma) -> bool {
        if let Ok(mut s_comp) = self.comp.lock() {
            s_comp.set_endpoint(set_gamma)
        } else { false }
    }

    fn set_limit(&mut self, min : Option<Gamma>, max : Option<Gamma>) {
        if let Ok(mut s_comp) = self.comp.lock() {
            s_comp.set_limit(min, max)
        }
    }

    fn apply_load_force(&mut self, force : Force) { // TODO: Add overload protection
        if let Ok(mut s_comp) = self.comp.lock() {
            s_comp.apply_load_force(force);
        }
    }

    fn apply_load_inertia(&mut self, inertia : Inertia) {
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