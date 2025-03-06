// Imports for Raspberry Pi related stuff.
use rppal::system::DeviceInfo;
// Imports for ratatui.
use std::io;


mod robo;
use robo::MyPi;

mod app;

// Consts for Hardware.
const GPIO_LED: u8 = 24;
const GPIO_US_TRIG: u8 = 17;
const GPIO_US_ECHO: u8 = 27;


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
                MyPi::Real(robo::real_pi::MyPiReal::new(GPIO_LED, GPIO_US_TRIG, GPIO_US_ECHO, 4. ))
                
            }
            else  
            {
                panic!("Unknown Raspberry Pi -> check if adjustments need to be made!")
            },
        Err(_) =>
            // Here we know that we are not on a Raspberry Pi-
            // Thus we return the Simulated Pi,
            MyPi::Sim(robo::sim_pi::MyPiSim::new(10.))
    };
    //  -----------------------------------------------
    // Here we start with the setup of the terminal UI.
    let mut terminal = ratatui::init();
    let app_result = app::App::new(my_pi).run(&mut terminal);
    ratatui::restore();
    app_result
}
