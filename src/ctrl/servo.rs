use crate::ctrl::pwm::PWMSignal;
use crate::data::ServoData;

pub struct ServoDriver
{
    pub pos : f32,
    pub data : ServoData,

    pub pwm : PWMSignal
}

impl ServoDriver 
{
    pub fn new(data : ServoData, pin_pwm : u16) -> Self {
        let mut pwm = PWMSignal::spawn(pin_pwm); 
        pwm.set_period(data.default_pulse(), data.cycle_time());

        ServoDriver {
            pos: data.default_pos(),
            data,

            pwm: pwm
        }
    }
}
