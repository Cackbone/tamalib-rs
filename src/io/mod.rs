mod button;
mod buzzer;
pub mod iobus;
mod screen;

pub use button::Button;
pub use buzzer::Buzzer;
pub use screen::{Screen, SCREEN_WIDTH, SCREEN_HEIGHT, ICONS_COUNT};

use crate::cpu::InputPin;
use crate::io::iobus::{Event, IOBus};
use std::cell::RefCell;
use std::rc::Rc;

pub struct IO {
    pub screen: Rc<RefCell<dyn Screen>>,
    pub buzzer: Box<dyn Buzzer>,
    pub bus: Rc<IOBus>,
}

impl IO {
    pub fn new(screen: Rc<RefCell<dyn Screen>>, buzzer: Box<dyn Buzzer>, bus: Rc<IOBus>) -> Self {
        Self {
            screen,
            buzzer,
            bus,
        }
    }

    pub fn set_buzzer_freq(&mut self, freq: u8) {
        self.buzzer
            .set_frequency(buzzer::BUZZER_FREQUENCIES[freq as usize]);
    }

    pub fn play_buzzer(&mut self, enabled: bool) {
        self.buzzer.play(enabled);
    }

    pub fn set_button(&mut self, button: Button, value: bool) {
        let pin = match button {
            Button::TAP => InputPin::K03,
            Button::LEFT => InputPin::K02,
            Button::MIDDLE => InputPin::K01,
            Button::RIGHT => InputPin::K00,
        };
        self.bus.emit(Event::ButtonPressed {
            pin: pin as u8,
            value: !value,
        });
    }

    pub fn set_screen_pin(&mut self, seg: usize, com: usize, value: bool) {
        if (screen::SEG_POS[seg] as usize) < SCREEN_WIDTH {
            self.screen
                .borrow_mut()
                .set_pixel(screen::SEG_POS[seg] as usize, com, value);
        } else {
            if seg == 8 && com < 4 {
                self.screen.borrow_mut().set_icon(com, value);
            } else if seg == 28 && com >= 12 {
                self.screen.borrow_mut().set_icon(com - 8, value);
            }
        }
    }
}
