use std::thread;
use std::error::Error;
use std::time::Duration;
// Imports for Raspberry Pi related stuff.
use rppal::gpio::Gpio;
use rppal::system::DeviceInfo;
use hc_sr04::HcSr04;

const GPIO_LED: u8 = 24;
const GPIO_US_TRIG: u8 = 17;
const GPIO_US_ECHO: u8 = 27;

trait RobotAccess {
    // Stuff for utrasonic sensor.
    fn get_hcsr04_dist(&mut self) -> Option<f32>;
}

struct MyPiReal {
    gpio_pin_led: rppal::gpio::OutputPin,
    pgio_us_hcsr04: HcSr04,
}

impl MyPiReal {
    fn new(gpio_led: u8, gpio_us_trig: u8, gpio_us_echo: u8) -> Self {
        MyPiReal { 
            gpio_pin_led: Gpio::new().unwrap().get(gpio_led).unwrap().into_output(), 
            pgio_us_hcsr04: HcSr04::new(gpio_us_trig, gpio_us_echo, None).unwrap()
        }
    }
}

impl RobotAccess for MyPiReal {
    fn get_hcsr04_dist(&mut self) -> Option<f32> {
        self.gpio_pin_led.set_high();
        match self.pgio_us_hcsr04.measure_distance(hc_sr04::Unit::Meters) {
            Ok(Some(dist)) => {
                self.gpio_pin_led.set_low();
                return Some(dist)},
            Ok(None) => {
                self.gpio_pin_led.set_low();
                return None}, // Out of Range.
            Err(t) => {
                println!("{}",t);
                self.gpio_pin_led.set_low();
                panic!("Error in hcsr04 dist measurement")},
        }
    }
}

struct MyPiSim {
    v: f32,
    max_range: f32,
}

impl MyPiSim {
    fn new(max_range: f32) -> Self {
        MyPiSim { 
            v: 0.,
            max_range
        }
    }
}

impl RobotAccess for MyPiSim {
    // Increase the value by one and return it. If it is larger than max_range set it to zero.
    fn get_hcsr04_dist(&mut self) -> Option<f32> {
        self.v += 0.01;
        if self.v > self.max_range {
            self.v = 0.;
        }
        return Some(self.v)
    }
}

// This Enum holds the either the real Pi or a Simulation of it.
enum MyPi {
    Real(MyPiReal),
    Sim(MyPiSim),
}

impl RobotAccess for MyPi {
    fn get_hcsr04_dist(&mut self) -> Option<f32> {
        match self {
            MyPi::Real(my_pi_real) => my_pi_real.get_hcsr04_dist(),
            MyPi::Sim(my_pi_sim) => my_pi_sim.get_hcsr04_dist(),
        }
    }
}


fn main() -> Result<(), Box<dyn Error>> {
    // Here we check if we are running on a raspberry Pi or a something else.
    let mut my_pi = match DeviceInfo::new() 
    {
        Ok(di) => 
            if di.model() == rppal::system::Model::RaspberryPi4B
            {
                // Now we know we are on a Raspberry Pi 4B.
                 println!("Device ID: {}.", di.model()); 
                // We init the GPIO structures
                MyPi::Real(MyPiReal::new(GPIO_LED, GPIO_US_TRIG, GPIO_US_ECHO))
            
            }
            else  
            {
                panic!("Unknown Raspberry Pi -> check if adjustments need to be made!")
            },
        Err(e) =>
            // Here we know that we are not on a Raspberry Pi-
            // Thus we return the Simulated Pi,
            MyPi::Sim(MyPiSim::new(10.))
    };
    // Init the GPIO access structs.

    loop {
        let k = my_pi.get_hcsr04_dist().unwrap();
        println!("{}",k);
        thread::sleep(Duration::from_secs(1));
    }
    
    Ok(())
}
