use super::*; 

pub struct ServoDriver
{
    pub pos : f32,
    pub pin_pwm : u16,

    // Thread
    pub thr : JoinHandle<()>,
    pub sender : Sender<f32>
}

impl ServoDriver 
{
    pub fn new(data : ServoData, pin_pwm : u16) -> Self {
        let pos = data.phi_max / 2.0;

        let mut sys_pwm = match SysFsGpioOutput::open(pin_pwm.clone()) {
            Ok(val) => RaspPin::Output(val),
            Err(_) => RaspPin::ErrPin
        };

        let (sender, recv) : (Sender<f32>, Receiver<f32>) = channel();

        let thr = thread::spawn(move || {
            let mut pos = data.default_pos();

            loop {
                match recv.try_recv() {
                    Ok(ang) => { pos = ang; },
                    _ => { }
                }

                ServoDriver::pulse(&data, &mut sys_pwm, pos);
            }
        }); 

        ServoDriver {
            pos,
            thr,
            sender,
            pin_pwm
        }
    }

    pub fn pulse(data : &ServoData, sys_pwm : &mut RaspPin, pos : f32) {
        match sys_pwm {
            RaspPin::Output(pin) => {
                let cycle = data.cycle_time();
                let pulse = data.pulse_time(pos);

                pin.set_high().unwrap();
                thread::sleep(Duration::from_secs_f32(pulse));
                pin.set_low().unwrap(); 
                thread::sleep(Duration::from_secs_f32(cycle - pulse));
            },
            _ => { }
        }
    }
}
