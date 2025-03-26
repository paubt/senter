use core::f64;

use std::time::Instant;
use std::time::Duration;
//use ratatui::crossterm;
use ratatui::widgets::canvas::Points;
use ratatui::widgets::canvas::Rectangle;
// Imports for ratatui.

extern crate nalgebra as na;

use std::io;
use ratatui::{
    layout::{Constraint, Layout, Rect},
    style::{Color, Modifier, Style, Stylize},
    symbols::{self, border},
    text::{Line, Text, Span},
    widgets::{canvas::Canvas, Block, Paragraph, Axis, Chart, Dataset},
    DefaultTerminal, Frame,
};
use ratatui::crossterm::event::{self, Event, KeyCode, KeyEventKind};

use crate::robo;
use robo::RobotAccess;

#[derive(Debug)]
pub enum SelectEnum {
    Velo,
    DelTime,
    MovStdDev,
    MeaStdDev,
}

impl SelectEnum {
    pub fn next(&self) -> SelectEnum {
        match self {
            SelectEnum::Velo => SelectEnum::DelTime,
            SelectEnum::DelTime => SelectEnum::MovStdDev,
            SelectEnum::MovStdDev => SelectEnum::MeaStdDev,
            SelectEnum::MeaStdDev => SelectEnum::Velo,
        }
    }

    pub fn previous(&self) -> SelectEnum{
        match self {
            SelectEnum::Velo => SelectEnum::MeaStdDev,
            SelectEnum::DelTime => SelectEnum::Velo,
            SelectEnum::MovStdDev => SelectEnum::DelTime,
            SelectEnum::MeaStdDev => SelectEnum::MovStdDev,
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
    pub belief: Option<Vec<f64>>,
    pub measurment: Option<Vec<f64>>,
    pub velo: f64,
    pub delta_t: Duration,
    pub mov_std_dev: f64,
    pub mea_std_dev: f64
}

impl<'a> World<'a> {
    pub fn new(name: &'a str, location: Option<f64>, door_list: Vec<f64>) -> World<'a> {
        let t = World {
            name: name, min: Some(0.), 
            max: Some(100.), location, door_list, 
            belief: None, measurment:None, 
            velo: 1., delta_t: Duration::from_secs(1), 
            mov_std_dev: 0.1, mea_std_dev: 1. };
        t
    }
    pub fn estimate_loc_after_next_step(&self) -> Option<f64> {
        match self.location {
            Some(l) => Some(l + self.velo*self.delta_t.as_secs_f64()),
            None => None,
        }
    }
}


#[derive(Debug)]
pub struct App<'a>{
    // Window A: Sensor data real time.
    // True if we want to record data with the sensor.
    // Stuff for Map display.
    world: World<'a>,
    // Stores the Access to the Hardware or its simulation.
    my_pi: robo::MyPi,
    // True if we want to close the app.
    exit: bool,
    // 
    sel_enu: SelectEnum,
}

impl<'a> App<'a> {
    pub fn new(my_pi: robo::MyPi) -> Self {
        let mut a = App {
            world: World::new("world", None, vec![(10.),(20.),(75.)]),
            my_pi,
            exit: false,
            sel_enu: SelectEnum::Velo};
        a.world.location = Some(a.my_pi.robot_position());
        a.world.measurment = Some(a.my_pi.robot_measurement());
        a.world.belief = Some(a.my_pi.robot_belief());
        a
    }

    pub fn run(&mut self, terminal: &mut DefaultTerminal) -> io::Result<()> {
        // This is the rate of update.
        let tick_rate = Duration::from_millis(125);
        // This stores the time of last update.
        let last_tick = Instant::now();
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
                            // KeyCode::Char('') => self.deactivate_sensor(),
                            // KeyCode::Char('e') => self.activate_sensor(),
                            KeyCode::Enter => self.next_step(),
                            KeyCode::Up => self.select_previouse_state(),
                            KeyCode::Down => self.select_next_state(),
                            KeyCode::Left => self.value_decrease_from_selected_state(),
                            KeyCode::Right => self.value_increase_from_selected_state(),
                            _ => {}
                        }
                    }
                    _ => {}
                }
            }
        }
        Ok(())
    }

    pub fn draw(&self, frame: &mut Frame) {
        let [left, right] = Layout::horizontal([Constraint::Fill(1), Constraint::Fill(2)]).areas(frame.area());

        self.render_info_box(frame, left);
        // 0 => self.render_sensor_data(frame, right),
        let [right_top, right_mid, right_bot] = Layout::vertical([Constraint::Length(4), Constraint::Fill(1), Constraint::Fill(1)]).areas(right);
        
        let [_, actual_map] = Layout::horizontal([Constraint::Length(5), Constraint::Fill(1)]).areas(right_top);
        self.render_map(frame, actual_map);
        self.render_measurement(frame, right_mid);
        self.render_belief(frame, right_bot);
        
    }
    
    fn render_info_box(&self, frame: &mut Frame, area: Rect) {
        let title = Line::from(" Senter ".bold());
        let instructions = Line::from(vec![
            " Select/Change ".into(),
            "<Arrows>".blue().bold(),
            " Next Step ".into(),
            "<Enter>".blue().bold(),
            " Quit ".into(),
            "<Q> ".blue().bold(),
        ]);
        let block = Block::bordered()
            .title(title.centered())
            .title_bottom(instructions.centered())
            .border_set(border::PLAIN);
        
        let mut lines = vec![Line::from(vec!["Position: x=".into(),  self.world.location.unwrap_or(-1.).to_string().yellow().into()]),
            Line::from(vec!["Velocity: v= ".into(), self.world.velo.to_string().yellow().into(), "ms".into()]),
            Line::from(vec!["Delta time: Δt=".into(), self.world.delta_t.as_secs_f64().to_string().yellow().into(), "s".into()]),
            Line::from(vec!["Movement standard deviation: σ_mov=".into(), self.world.mov_std_dev.to_string().yellow().into(), "".into()]),
            Line::from(vec!["Measurement standard deviation: σ_obs=".into(), self.world.mea_std_dev.to_string().yellow().into(), "".into()]),
            Line::from(vec!["Estimated position next step: new_x=".into(), self.world.estimate_loc_after_next_step().unwrap_or(-1.0).to_string().yellow().into(), "".into()])
            ];
        // Underline the line that is currently editable.
        let x = match self.sel_enu {
            SelectEnum::Velo => 1,
            SelectEnum::DelTime => 2,
            SelectEnum::MovStdDev => 3,
            SelectEnum::MeaStdDev => 4,
        };
        lines[x] = lines[x].clone().underlined();
        let counter_text: Text<'_> = Text::from(lines);
            
        let p = Paragraph::new(counter_text)
            .centered()
            .block(block);
            
        frame.render_widget(p, area);
    }

    fn render_map(&self, frame: &mut Frame, area: Rect) {       
        let map = Canvas::default()
            .block(Block::bordered().title(self.world.name))
            .x_bounds([area.x as f64, (area.x + area.width)  as f64])
            .y_bounds([area.y as f64, (area.y + area.height)  as f64])
            .paint(|ctx| {
                let y_door = area.y as f64 + ((area.height as f64 / 3.) * 2.);
                let y_rob = area.y as f64 + ((area.height as f64 / 3.));
                match self.world.door_list.is_empty() {
                    false => {
                        match self.world.location {
                            Some(l) => {
                                // Convert Coords to display size and offset.
                                let resized_loc: f64 = 
                                    area.x as f64 + (area.width as f64)*(l-self.world.min.unwrap())/(self.world.max.unwrap()- self.world.min.unwrap());
                                // Display as Point.
                                ctx.draw(&Points{ coords: &vec![(resized_loc, y_rob)], color: Color::Gray });
                            },
                            None => (),
                        }
                        // Same for wall points.
                        let r_w = 1.;
                        let r_h = 1.;
                        let resized_door_list: Vec<Rectangle> = self.world.door_list.iter().map(|d: &f64| {
                            Rectangle { 
                                x: area.x as f64 + (area.width as f64)*((d-self.world.min.unwrap())/(self.world.max.unwrap() - self.world.min.unwrap())) - r_w/2., 
                                y: y_door , 
                                width: r_w, 
                                height: r_h, 
                                color: Color::Blue }

                        }).collect();

                        resized_door_list.iter().for_each(|r| ctx.draw(r));
                        
                    
                    },
                    true => (),
                }
            });
        frame.render_widget(map, area);
    }

    fn render_measurement(&self, frame: &mut Frame, area: Rect) {
        match &self.world.measurment {
            Some(m) => {
                let max_y  = m.iter().fold(f64::MIN, |acc, x| f64::max(acc, *x) );
                let mid_y = max_y/2.;
                let x_labels = vec![
                    Span::styled(
                        format!("{}", 0),
                        Style::default().add_modifier(Modifier::BOLD),
                    ),
                    Span::raw(format!("{}", m.len() as f64 / 2.0)),
                    Span::styled(
                        format!("{}", m.iter().len() as f64),
                        Style::default().add_modifier(Modifier::BOLD),
                    )];
                let d = m.iter().enumerate().map(|(i,x)| (i as f64,*x )).clone().collect::<Vec<(f64,f64)>>();
                let datasets = vec![
                    Dataset::default()
                        //.name("p(z|m)")
                        .marker(symbols::Marker::Braille)
                        .style(Style::default().fg(Color::Cyan))
                        .data(&d)];
                let chart = Chart::new(datasets)
                    .block(Block::bordered().title("Measurement"))
                    .x_axis(
                        Axis::default()
                            .title("x")
                            .style(Style::default().fg(Color::Gray))
                            .labels(x_labels)
                            .bounds([0., 100.]),
                    )
                    .y_axis(
                        Axis::default()
                            .title("p(z|x)")
                            .style(Style::default().fg(Color::Gray))
                            .labels(["0".bold(),
                                mid_y.to_string().chars().take(4).collect::<String>().into(), 
                                max_y.to_string().chars().take(4).collect::<String>().bold()])
                            .bounds([0., max_y]),
                    );

                frame.render_widget(chart, area);
                
            },
            None => {
                let title = Line::from("Measurement");
                let block = Block::bordered()
                    .title(title.left_aligned())
                    .border_set(border::PLAIN);
                let counter_text: Text<'_> = Text::from(
                    vec![Line::from(vec!["No Measurement yet".into()])]);  
                
                let p = Paragraph::new(counter_text)
                    .centered()
                    .block(block);

                frame.render_widget(p, area);
            },
        }
    }
    
    fn render_belief(&self, frame: &mut Frame, area: Rect) {
        match &self.world.belief {
            Some(b) => {
                let max_y  = b.iter().fold(f64::MIN, |acc, x| f64::max(acc, *x) );
                let mid_y = max_y/2.;
                let x_labels = vec![
                    Span::styled(
                        format!("{}", 0),
                        Style::default().add_modifier(Modifier::BOLD),
                    ),
                    Span::raw(format!("{}", b.len() as f64 / 2.0)),
                    Span::styled(
                        format!("{}", b.len() as f64),
                        Style::default().add_modifier(Modifier::BOLD),
                    )];
                let d = b.iter().enumerate().map(|(i,x)| (i as f64,*x)).clone().collect::<Vec<(f64,f64)>>();
                let datasets = vec![
                    Dataset::default()
                        .name("Belief")
                        .marker(symbols::Marker::Braille)
                        .style(Style::default().fg(Color::Cyan))
                        .data(&d)];
                let chart = Chart::new(datasets)
                    .block(Block::bordered().title("Belief"))
                    .x_axis(
                        Axis::default()
                            .title("x")
                            .style(Style::default().fg(Color::Gray))
                            .labels(x_labels)
                            .bounds([0., 100.]),
                    )
                    .y_axis(
                        Axis::default()
                            .title("bel(x)")
                            .style(Style::default().fg(Color::Gray))
                            .labels(["0".bold(),
                            mid_y.to_string().chars().take(4).collect::<String>().into(), 
                            max_y.to_string().chars().take(4).collect::<String>().bold()])
                        .bounds([0., max_y]),
                );


                frame.render_widget(chart, area);
                
            },
            None => {
                let title = Line::from("Belief bel(x)");
                let block = Block::bordered()
                    .title(title.left_aligned())
                    .border_set(border::PLAIN);
                let counter_text: Text<'_> = Text::from(
                    vec![Line::from(vec!["No Belief yet".into()])]);  
                
                let p = Paragraph::new(counter_text)
                    .centered()
                    .block(block);

                frame.render_widget(p, area);
            },
        }
    }
    
    
    fn exit(&mut self) {
        self.exit = true;
    }
    
    fn next_step(&mut self) {
        // First send the updated values.
        self.my_pi.set_velo(self.world.velo);
        self.my_pi.set_delta_t(self.world.delta_t);
        self.my_pi.set_mov_std(self.world.mov_std_dev);
        self.my_pi.set_mae_std(self.world.mea_std_dev);
        // Invoke the actuall next step.
        self.my_pi.next_step();
        // Update values of the world after the step;
        self.world.location = Some(self.my_pi.robot_position());
        self.world.belief = Some(self.my_pi.robot_belief());
        self.world.measurment = Some(self.my_pi.robot_measurement());
    }
    
    fn select_previouse_state(&mut self) {
        self.sel_enu = self.sel_enu.previous()
    }
    
    fn select_next_state(&mut self) {
        self.sel_enu = self.sel_enu.next()
    }
    
    fn value_decrease_from_selected_state(&mut self) {
        match self.sel_enu {
            SelectEnum::Velo => self.world.velo = self.world.velo - 0.5 ,
            SelectEnum::DelTime => self.world.delta_t = self.world.delta_t - Duration::from_secs_f64(0.5),
            SelectEnum::MovStdDev => self.world.mov_std_dev = self.world.mov_std_dev - 0.5 ,
            SelectEnum::MeaStdDev => self.world.mea_std_dev = self.world.mea_std_dev - 0.5 ,
        }
    }
    
    fn value_increase_from_selected_state(&mut self) {
        match self.sel_enu {
            SelectEnum::Velo => self.world.velo = self.world.velo + 0.5 ,
            SelectEnum::DelTime => self.world.delta_t = self.world.delta_t + Duration::from_secs_f64(0.5) ,
            SelectEnum::MovStdDev => self.world.mov_std_dev = self.world.mov_std_dev + 0.5 ,
            SelectEnum::MeaStdDev => self.world.mea_std_dev = self.world.mea_std_dev + 0.5 ,
        }
    }

}
