use core::f64;

use nalgebra::{Vector2, Vector3};


extern crate nalgebra as na;


pub enum RobotStartBelief {
    // Startposition is as point mass at the given location.
    PointMass(f64),
}

pub trait RobotAccess {
    // Stuff for utrasonic sensor. This sensor point straigth ahead.
    fn get_hcsr04_dist(&mut self) -> Option<f64>;
    fn get_hcsr04_max_range(&self) -> f64;
    // Stuff for localization.
    fn get_map(&self) -> Vec<f64>;
    fn set_map(&mut self,map: Vec<f64>);
    //
    fn set_robot_position(&mut self, real_robot_position: f64);
    fn set_robot_belief(&mut self, robot_start_belief: RobotStartBelief);
    fn robot_position(&self) -> f64;
    // wheel velocity max and min for both sides.
    fn velo_max(&self) -> f64;
    fn velo_min(&self) -> f64;
    fn set_velo(&mut self, v: f64);
    fn get_velo(&self) -> f64;
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
    pub struct MyRobo {
        // distance between wheel
        diff_dive_const_l: f64,
        // where the robot beliefs it is. (x y theta)^T
        robot_position: Vector3<f64>,

    }

    #[derive(Debug)]
    pub struct MyPiSim {
        max_range: f64,
        map: Vec<f64>,
        // real_pose: (x y theta)^T
        robot_pose: f64,
        // action: (v_l v_r)^T
        belief: Vec<Vec<f64>>,
        max_velo: f64,
        min_velo: f64,
        velo: f64,
    }

    impl MyPiSim {
        pub fn new(max_range: f64, map: Vec<f64>) -> Self {

            let mut t = MyPiSim {
                max_range,
                map: Vec::new(),
                belief: Vec::new(),
                robot_pose: 0.,
                max_velo: 10.,
                min_velo: -10.,
                velo: 0.,
            };
            
            t.set_map(map);
            t
        }
    }

    impl super::RobotAccess for MyPiSim {
        // Increase the value by one and return it. If it is larger than max_range set it to zero.
        fn get_hcsr04_dist(&mut self) -> Option<f64> {
            
            self.velo += 0.2;
            if self.velo > self.max_range {
                self.velo = 0.;
            }
            return Some(self.velo)
        }
        
        fn get_hcsr04_max_range(&self) -> f64 {
            self.max_range
        }
        
        fn get_map(&self) -> Vec<f64> {
            self.map.clone()
        }
        
        fn set_map(&mut self,map: Vec<f64>) {
            self.map = map;
        }
        
        fn set_robot_position(&mut self, real_robot_position: f64) {
            self.robot_pose = real_robot_position;
        }
        
        fn set_robot_belief(&mut self, robot_start_belief: super::RobotStartBelief) {
            todo!()
        }
        
        fn robot_position(&self) -> f64 {
            self.robot_pose
        }
        
        fn velo_max(&self) -> f64 {
            self.max_velo
        }
        
        fn velo_min(&self) -> f64 {
            self.min_velo
        }
        
        fn set_velo(&mut self, v: f64) {
            self.velo = v;
        }

        fn get_velo(&self) -> f64 {
            self.velo
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
    
    fn get_map(&self) -> Vec<f64> {
        match self {
            MyPi::Real(my_pi_real) => my_pi_real.get_map(),
            MyPi::Sim(my_pi_sim) => my_pi_sim.get_map(),
        }
    }
    
    fn set_map(&mut self,map: Vec<f64>) {
        match self {
            MyPi::Real(my_pi_real) => my_pi_real.set_map(map),
            MyPi::Sim(my_pi_sim) => my_pi_sim.set_map(map),
        }
    }
    
    fn set_robot_position(&mut self, real_robot_position: f64) {
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
    
    fn robot_position(&self) -> f64 {
        match self {
            MyPi::Real(my_pi_real) => my_pi_real.robot_position(),
            MyPi::Sim(my_pi_sim) => my_pi_sim.robot_position(),
        }
    }
    
    fn velo_max(&self) -> f64 {
        match self {
            MyPi::Real(my_pi_real) => my_pi_real.velo_max(),
            MyPi::Sim(my_pi_sim) => my_pi_sim.velo_max(),
        }
    }
    
    fn velo_min(&self) -> f64 {
        match self {
            MyPi::Real(my_pi_real) => my_pi_real.velo_min(),
            MyPi::Sim(my_pi_sim) => my_pi_sim.velo_min(),
        }
    }
    
    fn set_velo(&mut self, v: f64) {
        match self {
            MyPi::Real(my_pi_real) => my_pi_real.set_velo(v),
            MyPi::Sim(my_pi_sim) => my_pi_sim.set_velo(v),
        }
    }
    
    fn get_velo(&self) -> f64 {
        match self {
            MyPi::Real(my_pi_real) => my_pi_real.get_velo(),
            MyPi::Sim(my_pi_sim) => my_pi_sim.get_velo(),
        }
    }
}

