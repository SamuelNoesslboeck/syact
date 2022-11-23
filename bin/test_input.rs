use gpio::{GpioIn, GpioOut, sysfs::*};

fn main() {
    let mut pin = SysFsGpioInput::open(25).expect("Could not open pin");
    
    let mut pin_rec = false;

    loop {
        let read_val = pin.read_value().unwrap() == gpio::GpioValue::High;

        if pin_rec && (!read_val) {
            pin_rec = false;

            println!("Input deactivated! {}", read_val);
        } else if (!pin_rec) && read_val {
            pin_rec = true;

            println!("Input activated! {}", read_val);
        }
    }
}