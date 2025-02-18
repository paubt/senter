use std::thread;
use std::error::Error;
use std::time::Duration;

use rppal::gpio::Gpio;
use rppal::system::DeviceInfo;

use hc_sr04::{HcSr04};

const GPIO_LED: u8 = 24;


fn main() -> Result<(), Box<dyn Error>> {
    println!("Hello, world!");

    println!("Device ID: {}.", DeviceInfo::new()?.model());

    let mut pin = Gpio::new()?.get(GPIO_LED)?.into_output();
    let mut  ultraschall = HcSr04::new(17, 27, None).unwrap();
    // Blink the LED by setting the pin's logic level high for 500 ms.

    // pin.set_high();
    // thread::sleep(Duration::from_millis(500));
    // pin.set_low();
    // thread::sleep(Duration::from_millis(500));

    loop {
        pin.set_high();
        match ultraschall.measure_distance(hc_sr04::Unit::Centimeters) {
            Ok(Some(dist)) => println!("{}cm", dist),
            Ok(None) => println!("Out of Range"),
            Err(t) => println!("{}",t),
        }
        pin.set_low();
        thread::sleep(Duration::from_secs(1));
    }
    
    Ok(())
}
