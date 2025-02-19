use std::thread;
use std::time::Duration;
use std::collections::VecDeque;
// Imports for Raspberry Pi related stuff.
use rppal::gpio::Gpio;
use rppal::system::DeviceInfo;
use hc_sr04::HcSr04;
// Imports for ratatui.
use std::io;
use ratatui::{
    buffer::Buffer,
    layout::Rect,
    style::Stylize,
    symbols::border,
    text::{Line, Text},
    widgets::{Block, Paragraph, Widget},
    DefaultTerminal, Frame,
};
use ratatui::crossterm::event::{self, Event, KeyCode, KeyEvent, KeyEventKind};

// Consts for Hardware.
const GPIO_LED: u8 = 24;
const GPIO_US_TRIG: u8 = 17;
const GPIO_US_ECHO: u8 = 27;
// Consts for Ratatui.
const SIZE_RINGBUFF_DIST: usize = 50;

trait RobotAccess {
    // Stuff for utrasonic sensor.
    fn get_hcsr04_dist(&mut self) -> Option<f32>;
}

#[derive(Debug)]
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

#[derive(Debug)]
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

#[derive(Debug)]
// This Enum holds the either the real Pi or a Simulation of it.
enum MyPi {
    Real(MyPiReal),
    Sim(MyPiSim),
}
// Here we forward the call to a secific struct that is inside the enum.
impl RobotAccess for MyPi {
    fn get_hcsr04_dist(&mut self) -> Option<f32> {
        match self {
            MyPi::Real(my_pi_real) => my_pi_real.get_hcsr04_dist(),
            MyPi::Sim(my_pi_sim) => my_pi_sim.get_hcsr04_dist(),
        }
    }
}



#[derive(Debug)]
pub struct App {
    // Ring buffer that pops at the end when inserting something at the beginning.
    ring_buf: VecDeque<f32>,
    mean: f32,
    my_pi: MyPi,
    exit: bool,
}

impl App {
    fn new(my_pi: MyPi) -> Self {
        App { 
            ring_buf: VecDeque::from(vec![0.; SIZE_RINGBUFF_DIST]),
            mean: 0.,
            my_pi,
            exit: false }
    }

    fn run(&mut self, terminal: &mut DefaultTerminal) -> io::Result<()> {
        while !self.exit {
            terminal.draw(|frame| self.draw(frame))?;
            self.handle_events()?;
        }
        Ok(())
    }
    
    fn draw(&self, frame: &mut Frame) {
        frame.render_widget(self, frame.area());
    }

    fn handle_events(&mut self) -> io::Result<()> {
        match event::read()? {
            // it's important to check that the event is a key press event as
            // crossterm also emits key release and repeat events on Windows.
            Event::Key(key_event) if key_event.kind == KeyEventKind::Press => {
                self.handle_key_event(key_event)
            }
            _ => {}
        };
        Ok(())
    }
    fn handle_key_event(&mut self, key_event: KeyEvent) {
        match key_event.code {
            KeyCode::Char('q') => self.exit(),
            KeyCode::Enter => self.new_meas_ring_buff(),
            _ => {}
        }
    }
    fn exit(&mut self) {
        self.exit = true;
    }

    fn new_meas_ring_buff(&mut self) {
        let t = self.ring_buf.pop_back();
        match self.my_pi.get_hcsr04_dist() {
            Some(d) => self.ring_buf.push_front(d),
            None => self.ring_buf.push_back(t.unwrap()),
        }
        let mut s:f32 = 0.;
        for v in &self.ring_buf {
            s += v
        }
        self.mean = s / SIZE_RINGBUFF_DIST as f32;
        // We also need to recalculate the mean
    }
}

impl Widget for &App {
    fn render(self, area: Rect, buf: &mut Buffer) {
        let title = Line::from(" Senter ".bold());
        let instructions = Line::from(vec![
            " Make new Measurements ".into(),
            "<Enter>".blue().bold(),
            " Quit ".into(),
            "<Q> ".blue().bold(),
        ]);
        let block = Block::bordered()
            .title(title.centered())
            .title_bottom(instructions.centered())
            .border_set(border::THICK);

        let counter_text = Text::from(vec![Line::from(vec![
            "mean: ".into(),
            self.mean.to_string().yellow(),
        ])]);
        
        Paragraph::new(counter_text)
            .centered()
            .block(block)
            .render(area, buf);
    }
}

fn main() -> io::Result<()> {
    //  -----------------------------------------------
    // Here we start with the Hardward setup.
    // Here we check if we are running on a raspberry Pi or a something else.
    // Either way we get a MyPi object.
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
        Err(_) =>
            // Here we know that we are not on a Raspberry Pi-
            // Thus we return the Simulated Pi,
            MyPi::Sim(MyPiSim::new(10.))
    };
    //  -----------------------------------------------
    // Here we start with the setup of the terminal UI.
    let mut terminal = ratatui::init();
    let app_result = App::new(my_pi).run(&mut terminal);
    ratatui::restore();
    app_result

    // loop {
    //     let k = my_pi.get_hcsr04_dist().unwrap();
    //     println!("{}",k);
    //     thread::sleep(Duration::from_secs(1));
    // }
    
}
