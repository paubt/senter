
pub trait RobotAccess {
    // Stuff for utrasonic sensor.
    fn get_hcsr04_dist(&mut self) -> Option<f64>;
    fn get_hcsr04_max_range(&self) -> f64;
}
pub mod real_pi {

    use hc_sr04::{HcSr04, Unit};
    use rppal::gpio::Gpio;
    #[derive(Debug)]
    pub struct MyPiReal {
        gpio_pin_led: rppal::gpio::OutputPin,
        pgio_us_hcsr04: HcSr04,
        max_range: f64,
    }

    impl MyPiReal {
        pub fn new(gpio_led: u8, gpio_us_trig: u8, gpio_us_echo: u8, max_range: f64) -> Self {
            MyPiReal { 
                gpio_pin_led: Gpio::new().unwrap().get(gpio_led).unwrap().into_output(), 
                pgio_us_hcsr04: HcSr04::new(gpio_us_trig, gpio_us_echo, None).unwrap(),
                max_range
            }
        }
    }

    impl super::RobotAccess for MyPiReal {
        fn get_hcsr04_dist(&mut self) -> Option<f64> {
            self.gpio_pin_led.set_high();
            match self.pgio_us_hcsr04.measure_distance(hc_sr04::Unit::Meters) {
                Ok(Some(dist)) => {
                    self.gpio_pin_led.set_low();
                    return Some(dist as f64)},
                Ok(None) => {
                    self.gpio_pin_led.set_low();
                    return None}, // Out of Range.
                Err(t) => {
                    println!("{}",t);
                    self.gpio_pin_led.set_low();
                    panic!("Error in hcsr04 dist measurement")},
            }
        }
        
        fn get_hcsr04_max_range(&self) -> f64 {
            self.max_range as f64
        }
    }
}

pub mod sim_pi {
    #[derive(Debug)]
    pub struct MyPiSim {
        v: f64,
        max_range: f64,
    }

    impl MyPiSim {
        pub fn new(max_range: f64) -> Self {
            MyPiSim { 
                v: 0.,
                max_range
            }
        }
    }

    impl super::RobotAccess for MyPiSim {
        // Increase the value by one and return it. If it is larger than max_range set it to zero.
        fn get_hcsr04_dist(&mut self) -> Option<f64> {
            self.v += 0.2;
            if self.v > self.max_range {
                self.v = 0.;
            }
            return Some(self.v)
        }
        
        fn get_hcsr04_max_range(&self) -> f64 {
            self.max_range
        }
    }
}
#[derive(Debug)]
// This Enum holds the either the real Pi or a Simulation of it.
pub enum MyPi {
    Real(real_pi::MyPiReal),
    Sim(sim_pi::MyPiSim),
}   
// Here we forward the call to a secific struct that is inside the enum.
impl RobotAccess for MyPi {
    fn get_hcsr04_dist(&mut self) -> Option<f64> {
        match self {
            MyPi::Real(my_pi_real) => my_pi_real.get_hcsr04_dist(),
            MyPi::Sim(my_pi_sim) => my_pi_sim.get_hcsr04_dist(),
        }
    }
    
    fn get_hcsr04_max_range(&self) -> f64 {
        match self {
            MyPi::Real(my_pi_real) => my_pi_real.get_hcsr04_max_range(),
            MyPi::Sim(my_pi_sim) => my_pi_sim.get_hcsr04_max_range(),
        }
    }
}

