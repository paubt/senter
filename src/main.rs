use std::{thread, time::Instant};
use std::time::Duration;
use std::collections::VecDeque;
use ratatui::crossterm;
// Imports for Raspberry Pi related stuff.
use rppal::gpio::Gpio;
use rppal::system::DeviceInfo;
use hc_sr04::{HcSr04, Unit};
// Imports for ratatui.
use std::io;
use ratatui::{
    buffer::Buffer,
    layout::{Constraint, Layout, Rect},
    style::{Color, Modifier, Style, Stylize},
    symbols::{border, self, Marker},
    text::{Line, Text, Span},
    widgets::{Block, Paragraph, Widget, Axis, Chart, Dataset, GraphType, LegendPosition},
    DefaultTerminal, Frame,
};
use ratatui::crossterm::event::{self, Event, KeyCode, KeyEvent, KeyEventKind};

// Consts for Hardware.
const GPIO_LED: u8 = 24;
const GPIO_US_TRIG: u8 = 17;
const GPIO_US_ECHO: u8 = 27;
// Consts for Ratatui.
const SIZE_RINGBUFF_DIST: usize = 60;

trait RobotAccess {
    // Stuff for utrasonic sensor.
    fn get_hcsr04_dist(&mut self) -> Option<f64>;
    fn get_hcsr04_max_range(&self) -> f64;
}

#[derive(Debug)]
struct MyPiReal {
    gpio_pin_led: rppal::gpio::OutputPin,
    pgio_us_hcsr04: HcSr04,
    max_range: f64,
}

impl MyPiReal {
    fn new(gpio_led: u8, gpio_us_trig: u8, gpio_us_echo: u8, max_range: f64) -> Self {
        MyPiReal { 
            gpio_pin_led: Gpio::new().unwrap().get(gpio_led).unwrap().into_output(), 
            pgio_us_hcsr04: HcSr04::new(gpio_us_trig, gpio_us_echo, None).unwrap(),
            max_range
        }
    }
}

impl RobotAccess for MyPiReal {
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

#[derive(Debug)]
struct MyPiSim {
    v: f64,
    max_range: f64,
}

impl MyPiSim {
    fn new(max_range: f64) -> Self {
        MyPiSim { 
            v: 0.,
            max_range
        }
    }
}

impl RobotAccess for MyPiSim {
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

#[derive(Debug)]
// This Enum holds the either the real Pi or a Simulation of it.
enum MyPi {
    Real(MyPiReal),
    Sim(MyPiSim),
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

#[derive(Debug)]
pub struct App {
    // True if we want to record data with the sensor.
    sens_data: bool,
    // Ring buffer that pops at the end when inserting something at the beginning.
    ring_buf: VecDeque<(f64,f64)>,
    mean: f64,
    // Stores the Access to the Hardware or its simulation.
    my_pi: MyPi,
    // True if we want to close the app.
    exit: bool,
}

impl App {
    fn new(my_pi: MyPi) -> Self {
        App {
            sens_data: false,
            ring_buf: VecDeque::from(vec![0.; SIZE_RINGBUFF_DIST]
                .into_iter()
                .enumerate()
                .map(|(u, f)| (u as f64, f))
                .collect::<Vec<(f64,f64)>>()),
            mean: 0.,
            my_pi,
            exit: false }
    }

    fn run(&mut self, terminal: &mut DefaultTerminal) -> io::Result<()> {
        // This is the rate of update.
        let tick_rate = Duration::from_millis(125);
        // This stores the time of last update.
        let mut last_tick = Instant::now();
        // This is the main loop.
        // Here we check at the beginning if the exit flag is set.
        while !self.exit {

            terminal.draw(|frame| self.draw(frame))?;
            
            //
            let timeout = tick_rate.saturating_sub(last_tick.elapsed());
            // This is the event handler.
            // Here we wait the timeout duration if a keyevent is made. Only if one happen
            // then we call read to get the key pressed.  
            if event::poll(timeout)? {
                match event::read()? {
                    // it's important to check that the event is a key press event as
                    // crossterm also emits key release and repeat events on Windows.
                    Event::Key(key_event) if key_event.kind == KeyEventKind::Press => {
                        match key_event.code {
                            KeyCode::Char('q') => self.exit(),
                            KeyCode::Char('w') => self.deactivate_sensor(),
                            KeyCode::Char('e') => self.activate_sensor(),
                            _ => {}
                        }
                    }
                    _ => {}
                }
            }
            // If the time since the last update is larger than the tick rate
            // we need to get a new measurment.
            if last_tick.elapsed() >= tick_rate {
                if self.sens_data {
                    // remove the oldest element.
                    let (_, ov ) = self.ring_buf.pop_back().unwrap();
                    // Get the index of the newest element by getting the seconde newest 
                    // and add 1.
                    let idx = match self.ring_buf.front() {
                        Some((i, _)) => *i + 1.,
                        None => 0.,
                    };
                    // 
                    match self.my_pi.get_hcsr04_dist() {
                        Some(v) => self.ring_buf.push_front((idx ,v)),
                        None => self.ring_buf.push_front((idx,self.my_pi.get_hcsr04_max_range())),
                    }
                    self.mean = self.mean + (self.ring_buf.front().unwrap().1 - ov) / SIZE_RINGBUFF_DIST as f64
                }
                last_tick = Instant::now();
            }
        }
        Ok(())
    }

    fn draw(&self, frame: &mut Frame) {
        let [left, right] = Layout::horizontal([Constraint::Fill(1); 2]).areas(frame.area());
        self.render_animated_chart(frame, right);
    }
    
    fn render_animated_chart(&self, frame: &mut Frame, area: Rect) {
        let li = self.ring_buf.back().unwrap().0;
        let ri = self.ring_buf.front().unwrap().0;
        
        let mut t = self.ring_buf.clone();


        let x_labels = vec![
            Span::styled(
                format!("{}", li),
                Style::default().add_modifier(Modifier::BOLD),
            ),
            Span::raw(format!("{}", (li + ri) as f32 / 2.0)),
            Span::styled(
                format!("{}", ri),
                Style::default().add_modifier(Modifier::BOLD),
            ),
        ];
        let datasets = vec![
            Dataset::default()
                .name("data2")
                .marker(symbols::Marker::Dot)
                .style(Style::default().fg(Color::Cyan))
                .data(t.make_contiguous())
        ];

        let chart = Chart::new(datasets)
            .block(Block::bordered())
            .x_axis(
                Axis::default()
                    .title("X Axis")
                    .style(Style::default().fg(Color::Gray))
                    .labels(x_labels)
                    .bounds([li as f64, ri as f64]),
            )
            .y_axis(
                Axis::default()
                    .title("Y Axis")
                    .style(Style::default().fg(Color::Gray))
                    .labels(["0".bold(),"5".into(), "10".bold()])
                    .bounds([0., 10.0]),
            );

        frame.render_widget(chart, area);
    }

    fn handle_key_event(&mut self, key_event: KeyEvent) {

    }

    fn exit(&mut self) {
        self.exit = true;
    }

    fn activate_sensor(&mut self) {
        self.sens_data = true
    }

    fn deactivate_sensor(&mut self) {
        self.sens_data = false
    }

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

        let counter_text: Text<'_> = Text::from(vec![Line::from(vec![
            "mean: ".into(),
            self.mean.to_string().yellow(),
        ])]);   
        
        Paragraph::new(counter_text)
            .centered()
            .block(block)
            .render(area, buf);
    }
}

// impl Widget for &App {
//     fn render(self, area: Rect, buf: &mut Buffer) {
//         let title = Line::from(" Senter ".bold());
//         let instructions = Line::from(vec![
//             " Make new Measurements ".into(),
//             "<Enter>".blue().bold(),
//             " Quit ".into(),
//             "<Q> ".blue().bold(),
//         ]);
//         let block = Block::bordered()
//             .title(title.centered())
//             .title_bottom(instructions.centered())
//             .border_set(border::THICK);

//         let counter_text = Text::from(vec![Line::from(vec![
//             "mean: ".into(),
//             self.mean.to_string().yellow(),
//         ])]);
        
//         Paragraph::new(counter_text)
//             .centered()
//             .block(block)
//             .render(area, buf);
//     }
// }

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
                MyPi::Real(MyPiReal::new(GPIO_LED, GPIO_US_TRIG, GPIO_US_ECHO, 4. ))
                
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
}
