use embedded_hal::PwmPin;
use sylo::Direction;

use crate::act::asyn::AsyncActuator;

use super::pwm::SoftwarePWM;

impl<CW : PwmPin, CCW : PwmPin> AsyncActuator for sylo::DcMotor<CW, CCW> 
where
    CW::Duty : Copy + Default,
    CCW::Duty : Copy + Default + From<CW::Duty>
{
    type Duty = CW::Duty;

    fn drive(&mut self, dir : Direction, speed : Self::Duty) -> Result<(), crate::Error> {
        sylo::DcMotor::drive(self, dir, speed);
        Ok(())
    }

    #[inline(always)]
    fn dir(&self) -> Direction {
        sylo::DcMotor::dir(self)
    }

    #[inline(always)]
    fn speed(&self) -> Self::Duty {
        sylo::DcMotor::speed(self)
    }
}

// Lazy decleration
pub type DcMotor = sylo::DcMotor<SoftwarePWM, SoftwarePWM>;