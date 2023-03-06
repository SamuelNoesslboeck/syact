use std::sync::mpsc::{channel, Sender, Receiver};
use std::thread;

type CommsFunc<Ctrl, Msg, Res> = fn (&mut Ctrl, Msg) -> Res;

// Submodule
mod stepper;
pub use stepper::AsyncCtrl;
//

/// ### `AsyncComms`
/// Struct for managing async movements
/// 
/// ``
/// 
/// extern crate alloc;
/// use alloc::rc::Rc;
/// 
/// use core::f32::consts::PI;
/// 
/// use stepper_lib::{Component, StepperCtrl, StepperConst, UpdateFunc};
/// use stepper_lib::comp::asynchr::AsyncComp;
/// use stepper_lib::data::LinkedData;
/// use stepper_lib::units::*;
/// 
/// let mut ctrl = StepperCtrl::new_sim(StepperConst::GEN);
/// ctrl.link(Rc::new(LinkedData { u: 12.0, s_f: 1.5 }));
/// 
/// ctrl.drive_rel_async(Delta(4.0 * PI), Omega(2.0 * PI));
///
/// ctrl.await_inactive();
/// ```
#[derive(Debug)]
pub struct AsyncHandler<Msg: Send + 'static, Res: Send + 'static>
{
    pub thr : thread::JoinHandle<()>,
    sender : Sender<Option<Msg>>,
    receiver : Receiver<Res>
}

impl<Msg: Send + 'static, Res: Send + 'static> AsyncHandler<Msg, Res> {
    pub fn new<Ctrl: Send + 'static>(mut ctrl : Ctrl, comms_func : CommsFunc<Ctrl, Msg, Res>) -> Self {
        let (sender_com, receiver_thr) : (Sender<Option<Msg>>, Receiver<Option<Msg>>) = channel();
        let (sender_thr, receiver_com) : (Sender<Res>, Receiver<Res>) = channel();

        let thr = thread::spawn(move || {
            loop {
                match receiver_thr.recv() {
                    Ok(msg_opt) => 
                        match msg_opt {
                            Some(msg) => {
                                let res = comms_func(&mut ctrl, msg);
                
                                sender_thr.send(res).unwrap();
                            },
                            None => {
                                break;
                            }
                        },
                    Err(err) => {
                        println!("Error occured in thread! {}", err.to_string());
                    }
                }
            };
        });

        Self {
            thr, 
            sender: sender_com,
            receiver: receiver_com
        }
    }

    pub fn send_msg(&self, msg : Msg) {
        // Clean recv buffer
        loop {
            if self.receiver.try_recv().is_err() {
                break;
            }
        }

        self.sender.send(Some(msg)).expect("Thread crashed")
    }

    pub fn await_inactive(&self) -> Res {
        self.receiver.recv().expect("Recv failed")  // TODO: Improve error handling
    }

    pub fn kill(&self) -> &thread::JoinHandle<()> {
        if !self.thr.is_finished() {
            self.sender.send(None).unwrap_or(());
        }

        &self.thr
    }
}

impl<Msg: Send + 'static, Res: Send + 'static> Drop for AsyncHandler<Msg, Res> {
    fn drop(&mut self) {
        self.kill();
    }
}
