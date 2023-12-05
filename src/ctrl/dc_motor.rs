use embedded_hal::PwmPin;
use sylo::Direction;

use crate::{comp::asyn::AsyncComp, Setup};

use super::pwm::SoftwarePWM;

impl<CW : PwmPin, CCW : PwmPin> Setup for sylo::DcMotor<CW, CCW> 
where
    CW::Duty : Copy + Default,
    CCW::Duty : Copy + Default + From<CW::Duty>
{
    fn setup(&mut self) -> Result<(), crate::Error> {
        self.cw_pin_mut().enable();
        self.ccw_pin_mut().enable();
        Ok(())
    }
}

impl<CW : PwmPin, CCW : PwmPin> AsyncComp for sylo::DcMotor<CW, CCW> 
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