use core::f64;

use nalgebra::{Vector2, Vector3};


extern crate nalgebra as na;


pub enum RobotStartBelief {
    // Startposition is as point mass at the given location.
    PointMass(Vector3<f64>),
}

pub trait RobotAccess {
    // Stuff for utrasonic sensor. This sensor point straigth ahead.
    fn get_hcsr04_dist(&mut self) -> Option<f64>;
    fn get_hcsr04_max_range(&self) -> f64;
    // Stuff for localization.
    fn get_map(&self) -> Vec<Vector2<f64>>;
    fn set_map(&mut self,map: Vec<Vector2<f64>>);
    //
    fn set_robot_position(&mut self, real_robot_position: Vector3<f64>);
    fn set_robot_belief(&mut self, robot_start_belief: RobotStartBelief);
    fn robot_position(&self) -> Vector3<f64>;
    // wheel velocity max and min for both sides.
    fn wheel_velo_max(&self) -> f64;
    fn wheel_velo_min(&self) -> f64;
    fn set_wheel_velo(&mut self, left: f64, right: f64);
    fn get_wheel_velo(&self) -> (f64, f64);
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
        
        fn get_map(&self) -> Vec<nalgebra::Vector2<f64>> {
            todo!()
        }
        
        fn set_map(&mut self,map: Vec<nalgebra::Vector2<f64>>) {
            todo!()
        }
        
        fn set_robot_position(&mut self, real_robot_position: nalgebra::Vector3<f64>) {
            todo!()
        }
        
        fn set_robot_belief(&mut self, robot_start_belief: super::RobotStartBelief) {
            todo!()
        }
        
        fn robot_position(&self) -> nalgebra::Vector3<f64> {
            todo!()
        }
        
        fn wheel_velo_max(&self) -> f64 {
            todo!()
        }
        
        fn wheel_velo_min(&self) -> f64 {
            todo!()
        }
        
        fn set_wheel_velo(&mut self, left: f64, right: f64) {
            todo!()
        }
        
        fn get_wheel_velo(&self) -> (f64, f64) {
            todo!()
        }
    }
}

pub mod sim_pi {
    use nalgebra::{Vector2, Vector3};

    use super::RobotAccess;
    
    #[derive(Debug,Clone,Copy, PartialEq, Eq)]
    pub enum PositionType {
        Wall,
        Empty,
    }

    #[derive(Debug)]
    pub struct MyPiSim {
        v: f64,
        max_range: f64,
        map: Vec<Vec<PositionType>>,
        robot_position: Vector3<f64>,
        belief: Vec<Vec<f64>>,
        max_velo: f64,
        min_velo: f64,
        velo_left: f64,
        velo_rigth: f64,
    }

    impl MyPiSim {
        pub fn new(max_range: f64, map: Vec<Vector2<f64>>) -> Self {

            let mut t = MyPiSim {
                v: 0.,
                max_range,
                map: Vec::new(),
                belief: Vec::new(),
                robot_position: Vector3::new(0.,0., 0.),
                max_velo: 10.,
                min_velo: -10.,
                velo_left: 0.,
                velo_rigth: 0.,
            };
            
            t.set_map(map);
            t
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
        
        fn get_map(&self) -> Vec<nalgebra::Vector2<f64>> {
            let mut r = 0;
            let mut c = 0;
            let mut v = Vec::new();
            for row in &self.map {
                for col in row {
                    if *col == PositionType::Wall {
                        v.push(Vector2::new(r as f64, c as f64));
                    }
                    c += 1;
                }
                r += 1;
                c = 0;
            }
            v
        }
        
        fn set_map(&mut self,map: Vec<nalgebra::Vector2<f64>>) {
            let (minx,miny,maxx,maxy) = map.iter()
                .fold((f64::MAX, f64::MAX, f64::MIN, f64::MIN), |mut acc, v| {
                    acc.0 = acc.0.min(v.x);
                    acc.1 = acc.1.min(v.y);
                    acc.2 = acc.2.max(v.x);
                    acc.3 = acc.3.max(v.y);
                    acc
            });
            let row  = vec![PositionType::Empty; maxx.floor() as usize - minx.floor() as usize];
            let m  = vec![row; maxy.floor() as usize - miny.floor() as usize];
            self.map = m;
        }
        
        fn set_robot_position(&mut self, real_robot_position: nalgebra::Vector3<f64>) {
            self.robot_position = real_robot_position;
        }
        
        fn set_robot_belief(&mut self, robot_start_belief: super::RobotStartBelief) {
            todo!()
        }
        
        fn robot_position(&self) -> nalgebra::Vector3<f64> {
            self.robot_position
        }
        
        fn wheel_velo_max(&self) -> f64 {
            self.max_velo
        }
        
        fn wheel_velo_min(&self) -> f64 {
            self.min_velo
        }
        
        fn set_wheel_velo(&mut self, left: f64, right: f64) {
            self.velo_left = left;
            self.velo_rigth = right;
        }

        fn get_wheel_velo(&self) -> (f64, f64) {
            (self.velo_left, self.velo_rigth)
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
    
    fn get_map(&self) -> Vec<Vector2<f64>> {
        match self {
            MyPi::Real(my_pi_real) => my_pi_real.get_map(),
            MyPi::Sim(my_pi_sim) => my_pi_sim.get_map(),
        }
    }
    
    fn set_map(&mut self,map: Vec<Vector2<f64>>) {
        match self {
            MyPi::Real(my_pi_real) => my_pi_real.set_map(map),
            MyPi::Sim(my_pi_sim) => my_pi_sim.set_map(map),
        }
    }
    
    fn set_robot_position(&mut self, real_robot_position: Vector3<f64>) {
        match self {
            MyPi::Real(my_pi_real) => my_pi_real.set_robot_position(real_robot_position),
            MyPi::Sim(my_pi_sim) => my_pi_sim.set_robot_position(real_robot_position),
        }
    }
    
    fn set_robot_belief(&mut self, robot_start_belief: RobotStartBelief) {
        match self {
            MyPi::Real(my_pi_real) => my_pi_real.set_robot_belief(robot_start_belief),
            MyPi::Sim(my_pi_sim) => my_pi_sim.set_robot_belief(robot_start_belief),
        }
    }
    
    fn robot_position(&self) -> Vector3<f64> {
        match self {
            MyPi::Real(my_pi_real) => my_pi_real.robot_position(),
            MyPi::Sim(my_pi_sim) => my_pi_sim.robot_position(),
        }
    }
    
    fn wheel_velo_max(&self) -> f64 {
        match self {
            MyPi::Real(my_pi_real) => my_pi_real.wheel_velo_max(),
            MyPi::Sim(my_pi_sim) => my_pi_sim.wheel_velo_max(),
        }
    }
    
    fn wheel_velo_min(&self) -> f64 {
        match self {
            MyPi::Real(my_pi_real) => my_pi_real.wheel_velo_min(),
            MyPi::Sim(my_pi_sim) => my_pi_sim.wheel_velo_min(),
        }
    }
    
    fn set_wheel_velo(&mut self, left: f64, right: f64) {
        match self {
            MyPi::Real(my_pi_real) => my_pi_real.set_wheel_velo(left, right),
            MyPi::Sim(my_pi_sim) => my_pi_sim.set_wheel_velo(left, right),
        }
    }
    
    fn get_wheel_velo(&self) -> (f64, f64) {
        match self {
            MyPi::Real(my_pi_real) => my_pi_real.get_wheel_velo(),
            MyPi::Sim(my_pi_sim) => my_pi_sim.get_wheel_velo(),
        }
    }
}

