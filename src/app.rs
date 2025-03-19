use core::f64;
use std::char::MAX;
use std::collections::VecDeque;

use std::time::Instant;
use std::time::Duration;
use nalgebra::Vector2;
//use ratatui::crossterm;
use ratatui::widgets::canvas::Points;
// Imports for ratatui.

extern crate nalgebra as na;

use std::io;
use ratatui::{
    layout::{Constraint, Layout, Rect},
    style::{Color, Modifier, Style, Stylize},
    symbols::{self, border},
    text::{Line, Text, Span},
    widgets::{canvas::{Canvas, Shape, Painter}, Block, Paragraph, Axis, Chart, Dataset},
    DefaultTerminal, Frame,
};
use ratatui::crossterm::event::{self, Event, KeyCode, KeyEventKind};

use crate::robo;
use robo::RobotAccess;


// Consts for Ratatui.
const SIZE_RINGBUFF_DIST: usize = 60;

#[derive(Debug)]
pub struct TabsState<'a> {
    pub titles: Vec<&'a str>,
    pub index: usize,
}

impl<'a> TabsState<'a> {
    pub const fn new(titles: Vec<&'a str>) -> Self {
        Self { titles, index: 0 }
    }
    pub fn next(&mut self) {
        self.index = (self.index + 1) % self.titles.len();
    }

    pub fn previous(&mut self) {
        if self.index > 0 {
            self.index -= 1;
        } else {
            self.index = self.titles.len() - 1;
        }
    }
}

#[derive(Debug)]
pub struct World<'a> {
    pub name: &'a str,
    pub min: Option<f64>,
    pub max: Option<f64>,
    pub location: Option<f64>,
    pub door_list: Vec<f64>,
}

impl<'a> World<'a> {
    pub fn new(name: &'a str, location: Option<f64>, door_list: Vec<f64>) -> World<'a> {
        let mut t = World {name: name, min: None, max: None, location, door_list};
        t.update_min_max();
        t
    }
    // add a new wall point and update the min max values.
    pub fn add_wall_point(&mut self, new_wall_point: f64) {
        self.door_list.push(new_wall_point);
        self.update_min_max();
    }
    
    fn update_min_max(&mut self) {
        if self.door_list.is_empty() {
            self.max = None;
            self.min = None;
        }
        else {
            let t = self.door_list.iter()
                .fold((f64::MAX, f64::MAX, f64::MIN, f64::MIN), |mut acc, v| {
                    acc.0 = acc.0.min(v.x);
                    acc.1 = acc.1.min(v.y);
                    acc.2 = acc.2.max(v.x);
                    acc.3 = acc.3.max(v.y);
                    acc
            });
            self.min = Some(Vector2::new(t.0, t.1));
            self.max = Some(Vector2::new(t.2, t.3));
        }
    }

    pub fn remove_wall_point(&mut self, wall_point: Vector2<f64>) {
        match self.wall_list.iter()
            .find(|&&v| v == wall_point) {
                Some(_) => {
                    self.wall_list = self
                        .wall_list
                        .iter()
                        .filter(|&& v| v != wall_point)
                        .cloned()
                        .collect();
                    self.update_min_max();
                },
                None => (),
            }
    }
}

// This is so it can by drawn.
impl<'a> Shape for World<'a> {
    fn draw(&self, painter: &mut Painter) {
        for v in self.wall_list.clone() {
            painter.paint(v.x as usize, v.y as usize, Color::White);
        }
    }
}

// impl<'a> World<'a> {
//     pub fn resize_to_area(&mut self, area: Rect) -> Vec<(f64,f64)> {
//         self.wall_list.iter().map(|(x, y)| {
//             (*x, *y)
//         }).collect()
//     }
// }

#[derive(Debug)]
pub struct App<'a>{
    pub tabs: TabsState<'a>,
    // Window A: Sensor data real time.
    // True if we want to record data with the sensor.
    sens_data: bool,
    // Ring buffer that pops at the end when inserting something at the beginning.
    ring_buf: VecDeque<(f64,f64)>,
    mean: f64,
    // Stuff for Map display.
    world: World<'a>,
    // Stores the Access to the Hardware or its simulation.
    my_pi: robo::MyPi,
    // True if we want to close the app.
    exit: bool,
}

impl<'a> App<'a> {
    pub fn new(my_pi: robo::MyPi) -> Self {
        App {
            tabs: TabsState::new(vec!["Map", "Sensor"]),
            sens_data: false,
            ring_buf: VecDeque::from(vec![0.; SIZE_RINGBUFF_DIST]
                .into_iter()
                .enumerate()
                .map(|(u, f)| (u as f64, f))
                .collect::<Vec<(f64,f64)>>()),
            mean: 0.,
            // world: World { name: "small",min: (0.,0.),max: (39.,39.) , location: (2.,6.), wall_list: WALL_SMALL.to_vec()},
            // world: World::new("small", None,  WALL_SMALL.to_vec().iter().map(|(x,y)| Vector2::new(*x,*y)).collect::<Vec<Vector2<f64>>>()),
            world: World::new("big", None,  WALL_BIG.to_vec().iter().map(|(x,y)| Vector2::new(*x,*y)).collect::<Vec<Vector2<f64>>>()),
            //world: World { name: "big",min: (0.,0.),max: (99.,99.) , location: (5.,20.), wall_list: WALL_BIG.to_vec()},
            my_pi,
            exit: false }
    }

    pub fn run(&mut self, terminal: &mut DefaultTerminal) -> io::Result<()> {
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
                            KeyCode::Left => self.lower_tab(),
                            KeyCode::Right => self.raise_tab(), 
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

    pub fn draw(&self, frame: &mut Frame) {
        let [left, right] = Layout::horizontal([Constraint::Fill(1), Constraint::Fill(1)]).areas(frame.area());

        self.render_info_box(frame, left);
        // 0 => self.render_sensor_data(frame, right),

        match self.tabs.index {
            0 => self.render_map(frame, right),
            1 => self.render_map(frame, right),
            _ => panic!("unkown tab id")
        };
        
    }
    
    fn render_info_box(&self, frame: &mut Frame, area: Rect) {
        let title = Line::from(" Senter ".bold());
        let instructions = Line::from(vec![
            " Move ".into(),
            "<Arrows>".blue().bold(),
            " reset ".into(),
            "<r>".blue().bold(),
            " Quit ".into(),
            "<Q> ".blue().bold(),
        ]);
        let block = Block::bordered()
            .title(title.centered())
            .title_bottom(instructions.centered())
            .border_set(border::THICK);
        
        let (vl, vr) = self.my_pi.get_wheel_velo();
        let p = self.my_pi.robot_position();
        let counter_text: Text<'_> = Text::from(
            vec![Line::from(vec!["Position: x=".into(), p.x.to_string().yellow().into(), " y=".into(), p.y.to_string().yellow().into(), ]),
                 Line::from(vec!["Wheel velo:  x=".into(), vl.to_string().yellow(), " y=".into(), vr.to_string().yellow()])
                 ]);  
        
        let [left_top, left_bot] = Layout::vertical([Constraint::Fill(1), Constraint::Fill(1)]).areas(area);

        self.render_sensor_data(frame, left_bot);
        let p = Paragraph::new(counter_text)
            .centered()
            .block(block);

        frame.render_widget(p, left_top);
    }

    fn render_sensor_data(&self, frame: &mut Frame, area: Rect) {
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
                .name("hcsr04")
                .marker(symbols::Marker::Dot)
                .style(Style::default().fg(Color::Cyan))
                .data(t.make_contiguous())
        ];

        let chart = Chart::new(datasets)
            .block(Block::bordered().title("Sensor"))
            .x_axis(
                Axis::default()
                    .title("time")
                    .style(Style::default().fg(Color::Gray))
                    .labels(x_labels)
                    .bounds([li as f64, ri as f64]),
            )
            .y_axis(
                Axis::default()
                    .title("distance")
                    .style(Style::default().fg(Color::Gray))
                    .labels(["0".bold(),"5".into(), "10".bold()])
                    .bounds([0., 10.0]),
            );

        frame.render_widget(chart, area);
    }

    fn render_map(&self, frame: &mut Frame, area: Rect) {       
        let map = Canvas::default()
            .block(Block::bordered().title(self.world.name))
            .x_bounds([area.x as f64, (area.x + area.width)  as f64])
            .y_bounds([area.y as f64, (area.y + area.height)  as f64])
            .paint(|ctx| {
                match self.world.wall_list.is_empty() {
                    false => {
                        match self.world.location {
                            Some(l) => {
                                // Convert Coords to display size and offset.
                                let resized_loc: (f64, f64) = 
                                (area.x as f64 + (area.width as f64)*(l.x-self.world.min.unwrap().x)/(self.world.max.unwrap().x- self.world.min.unwrap().x),
                                area.y as f64 + (area.height as f64)*(l.y-self.world.min.unwrap().y)/(self.world.max.unwrap().y- self.world.min.unwrap().y));
                                // Display as Point.
                                ctx.draw(&Points{ coords: &vec![resized_loc], color: Color::White });
                            },
                            None => (),
                        }
                        // Same for wall points.
                        let resized_wall_list: Vec<(f64,f64)>= self.world.wall_list.iter().map(|v: &Vector2<f64>| {
                            (area.x as f64 + (area.width as f64)*(v.x-self.world.min.unwrap().x)/(self.world.max.unwrap().x - self.world.min.unwrap().x),
                             area.y as f64 + (area.height as f64)*(v.y-self.world.min.unwrap().y)/(self.world.max.unwrap().y - self.world.min.unwrap().y))
                        }).collect();
                        ctx.draw(&Points{ coords:&resized_wall_list, color: Color::White });
                    },
                    true => (),
                }
                // ctx.draw(&ratatui::widgets::canvas::Line {
                //     x1: area.x as f64,
                //     y1: area.y as f64,
                //     x2: (area.x + (area.width / 8 )) as f64,
                //     y2: (area.y + (area.height / 2)) as f64,
                //     color: Color::White,
                // });
                });
        frame.render_widget(map, area);
    }

    fn raise_tab(&mut self) {
        self.tabs.next();
    }
    
    fn lower_tab(&mut self) {
        self.tabs.previous();
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

}
