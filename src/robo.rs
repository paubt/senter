use std::time::Duration;

use nalgebra::{Vector2, Vector3};


extern crate nalgebra as na;


pub enum RobotStartBelief {
    // Startposition is as point mass at the given location.
    PointMass(f64),
    Uniform,
}

pub trait RobotAccess {
    // Stuff for utrasonic sensor. This sensor point straigth ahead.
    fn get_hcsr04_dist(&mut self) -> Option<f64>;
    fn get_hcsr04_max_range(&self) -> f64;
    // Stuff for localization.
    fn get_map(&self) -> Vec<(f64,u64)>;
    fn set_map(&mut self,map: Vec<(f64,u64)>);
    // robot stuff
    fn set_robot_position(&mut self, real_robot_position: f64);
    fn set_robot_belief(&mut self, robot_start_belief: RobotStartBelief);
    fn robot_position(&self) -> f64;
    fn robot_belief(&self) -> Vec<f64>;
    fn robot_measurement(&self) -> Vec<f64>;
    // wheel velocity max and min for both sides.
    fn velo_max(&self) -> f64;
    fn velo_min(&self) -> f64;
    fn set_velo(&mut self, v: f64);
    fn get_velo(&self) -> f64;
    fn set_mov_std(&mut self, s: f64);
    fn set_mae_std(&mut self, s: f64);
    fn set_delta_t(&mut self, d: Duration);
    // operate
    fn next_step(&mut self);
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
        
        fn get_map(&self) -> Vec<(f64,u64)> {
            todo!()
        }
        
        fn set_map(&mut self,map: Vec<(f64,u64)>) {
            todo!()
        }
        
        fn set_robot_position(&mut self, real_robot_position: f64) {
            todo!()
        }
        
        fn set_robot_belief(&mut self, robot_start_belief: super::RobotStartBelief) {
            todo!()
        }
        
        fn robot_position(&self) -> f64 {
            todo!()
        }
        
        fn velo_max(&self) -> f64 {
            todo!()
        }
        
        fn velo_min(&self) -> f64 {
            todo!()
        }
        
        fn set_velo(&mut self, v: f64) {
            todo!()
        }
        
        fn get_velo(&self) -> f64 {
            todo!()
        }
        
        fn robot_belief(&self) -> Vec<f64> {
            todo!()
        }
        
        fn robot_measurement(&self) -> Vec<f64> {
            todo!()
        }
        
        fn next_step(&mut self) {
            todo!()
        }
        
        fn set_mov_std(&mut self, s: f64) {
            todo!()
        }
        
        fn set_mae_std(&mut self, s: f64) {
            todo!()
        }
        
        fn set_delta_t(&mut self, d: std::time::Duration) {
            todo!()
        }
        
        
    }
}

pub mod sim_pi {
    use std::{f64::consts::PI, time::{Duration, Instant}};

    use nalgebra::{Vector2, Vector3};

    use super::RobotAccess;
    
    #[derive(Debug,Clone,Copy, PartialEq, Eq)]
    pub enum PositionType {
        Wall,
        Empty,
    }

    #[derive(Debug)]
    pub struct MyPiSim {
        // Environment
        map: Vec<(f64,u64)>,
        // Environment from Zero to size.
        map_size: u64,
        last_update: Instant, 
        // real robot
        pub movement_std: f64,
        pub measurement_std: f64,
        pub update_delta_t: Duration,
        // Robot
        robot_pose: f64,
        belief: Vec<f64>,
        max_velo: f64,
        min_velo: f64,
        velo: f64,
        motion_model_alpha: f64,
    }

    #[derive(Debug,PartialEq)]
    pub enum SensorReading {
        Door,
        Nothing,
    }

    fn prob_norm_dis(q: f64,s: f64) -> f64 {
        let denom: f64 = (2. * std::f64::consts::PI * s).powf(0.5);
        let num  = ( -0.5 * (q  / s) ).exp();
        num/denom
    }

    impl MyPiSim {
        pub fn new(map_size: u64, map: Vec<(f64,u64)>) -> Self {
            let mut t = MyPiSim {
                map: Vec::new(),
                map_size: map_size,
                last_update: Instant::now(),
                belief: Vec::new(),
                robot_pose: 20.,
                max_velo: 10.,
                min_velo: -10.,
                velo: 10.,
                update_delta_t: Duration::from_secs(1),
                movement_std: 1.,
                measurement_std: 1.,
                motion_model_alpha: 0.2,
            };
            t.set_map(map);
            t.set_robot_belief(super::RobotStartBelief::Uniform);
            t

        }
        fn run_grid_loc(&mut self, z: SensorReading) {
            // Table 8.1
            if self.velo.abs() > 0.0001 {
                // First to the Motion update.
                let mut bar_bel = Vec::new();
                for k in 0..self.map_size {
                    let mut t = 0.;
                    for i in 0..self.map_size {
                        t += self.belief[i as usize] * self.motion_model_velo(k as f64, self.velo, i as f64)
                    }
                    bar_bel.push(t);
                }
                let norm: f64= bar_bel.iter().sum();

                for k in 0..self.map_size {
                    self.belief[k as usize] = (1. / norm) * bar_bel[k as usize];
                }

                // then perception
                // let mut p_new: Vec<f64> = Vec::new();

                // for k in 0..self.map_size {
                //     p_new.push( bar_bel[k as usize] * self.measurement_model_z_at_x(k as f64, &z));
                // }
                // let norm: f64= p_new.iter().sum();

                // for k in 0..self.map_size {
                //     self.belief[k as usize] = (1. / norm) * p_new[k as usize];
                // }
            }
            // Only preceptual.
            else {

                let mut p_new: Vec<f64> = Vec::new();

                for k in 0..self.map_size {
                    p_new.push( self.belief[k as usize] * self.measurement_model_z_at_x(k as f64, &z));
                }
                let norm: f64= p_new.iter().sum();

                for k in 0..self.map_size {
                    self.belief[k as usize] = (1. / norm) * p_new[k as usize];
                }
            }
        }

        fn measurement_model_z_at_x(&self, x:f64, z: &SensorReading) -> f64 {
            let denom: f64 = (2.*PI* self.measurement_std.powi(2)).powf(0.5);
            // find closest and clac prob
            let d = self.map
                .iter()
                .max_by(|(a,s1),(b,s2)| {
                    if (a -  x).abs() > (b - x).abs() {
                        std::cmp::Ordering::Less
                    }
                    else if (a - x).abs() < (b - x).abs() {
                        std::cmp::Ordering::Greater
                    }
                    else {
                        std::cmp::Ordering::Equal
                    }
                }).unwrap();
            let num  = ( (-1. * (x-d.0).powi(2) ) / (2. * self.measurement_std.powi(2)) ).exp();
            let p_door_given_x = num / denom;
            // if we want the negative, we just negate the probabilty; 
            if *z == SensorReading::Nothing {
                return  1. - p_door_given_x
            }
            p_door_given_x
        }
        
        fn motion_model_velo(&self, to_pos:f64, velo:f64, from_pose:f64) -> f64 {
            // First calc the Velocity that the robot would have needed to get from 
            // from_pos to to_pos. 
            let v_hat = (to_pos - from_pose) / self.update_delta_t.as_secs_f64();
            // This ist the difference in v that would be needed to reach to_pos and the
            // actual v. 
            let v_err = (velo - v_hat).abs();
            // Calc the probablity
            prob_norm_dis(v_err, self.motion_model_alpha * velo.powi(2))
        }
    }

    impl super::RobotAccess for MyPiSim {
        // Increase the value by one and return it. If it is larger than max_range set it to zero.
        fn get_hcsr04_dist(&mut self) -> Option<f64> {
            
            // self.velo += 0.2;
            // if self.velo > self.max_range {
            //     self.velo = 0.;
            // }
            return None
        }
        
        fn get_hcsr04_max_range(&self) -> f64 {
            5. //self.max_range
        }
        
        fn get_map(&self) -> Vec<(f64,u64)> {
            self.map.clone()
        }
        
        fn set_map(&mut self,map: Vec<(f64,u64)>) {
            self.map = map;
        }
        
        fn set_robot_position(&mut self, real_robot_position: f64) {
            if real_robot_position >= 0. && real_robot_position <= self.map_size as f64 {
                self.robot_pose = real_robot_position;
            }
        }
        
        fn set_robot_belief(&mut self, robot_start_belief: super::RobotStartBelief) {
            match robot_start_belief {
                super::RobotStartBelief::PointMass(_) => todo!(),
                super::RobotStartBelief::Uniform => self.belief = vec![1. / self.map_size as f64; self.map_size as usize],
            }
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
        
        fn robot_belief(&self) -> Vec<f64> {
            self.belief.clone()
        }
        
        fn robot_measurement(&self) -> Vec<f64> {
            let t: Vec<f64> = (0..self.map_size)
                .map(|i| self.measurement_model_z_at_x(i as f64, &SensorReading::Door))
                .collect();
            let norm: f64 = t.iter().sum();
            t.iter().map(|p| p/norm).collect()
            
        }
        
        fn next_step(&mut self) {
            // Move the robot in the sim.
            self.robot_pose = self.robot_pose + self.update_delta_t.as_secs_f64() * self.velo;

            // Get a new sensor reading. We read a door if we are close to it. 
            let sensor_dist_threshold = 3.;
            let z: SensorReading = self.map.iter()
                .fold(SensorReading::Nothing,
                      |acc , (x,s)| if ((self.robot_pose - x).abs() < sensor_dist_threshold) {SensorReading::Door} else {acc}  );
            
            // Run the localization alog.
            self.run_grid_loc(z);
        }
        
        fn set_mov_std(&mut self, s: f64) {
            self.movement_std = s;
        }
        
        fn set_mae_std(&mut self, s: f64) {
            self.measurement_std = s;
        }
        
        fn set_delta_t(&mut self, d: Duration) {
            self.update_delta_t = d;
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
    
    fn get_map(&self) -> Vec<(f64,u64)> {
        match self {
            MyPi::Real(my_pi_real) => my_pi_real.get_map(),
            MyPi::Sim(my_pi_sim) => my_pi_sim.get_map(),
        }
    }
    
    fn set_map(&mut self,map: Vec<(f64,u64)>) {
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
    
    fn robot_belief(&self) -> Vec<f64> {
        match self {
            MyPi::Real(my_pi_real) => my_pi_real.robot_belief(),
            MyPi::Sim(my_pi_sim) => my_pi_sim.robot_belief(),
        }
    }
    
    fn robot_measurement(&self) -> Vec<f64> {
        match self {
            MyPi::Real(my_pi_real) => my_pi_real.robot_measurement(),
            MyPi::Sim(my_pi_sim) => my_pi_sim.robot_measurement(),
        }
    }
    
    fn next_step(&mut self) {
        match self {
            MyPi::Real(my_pi_real) => my_pi_real.next_step(),
            MyPi::Sim(my_pi_sim) => my_pi_sim.next_step(),
        }
    }
    
    fn set_mov_std(&mut self, s: f64) {
        match self {
            MyPi::Real(my_pi_real) => my_pi_real.set_mov_std(s),
            MyPi::Sim(my_pi_sim) => my_pi_sim.set_mov_std(s),
        }
    }
    
    fn set_mae_std(&mut self, s: f64) {
        match self {
            MyPi::Real(my_pi_real) => my_pi_real.set_mae_std(s),
            MyPi::Sim(my_pi_sim) => my_pi_sim.set_mae_std(s),
        }
    }
    
    fn set_delta_t(&mut self, d: Duration) {
        match self {
            MyPi::Real(my_pi_real) => my_pi_real.set_delta_t(d),
            MyPi::Sim(my_pi_sim) => my_pi_sim.set_delta_t(d),
        }
    }
}

