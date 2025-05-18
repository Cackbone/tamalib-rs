use std::cell::RefCell;
use std::rc::Rc;

#[derive(Debug, Clone)]
pub enum Event {
    ButtonPressed { pin: u8, value: bool },
    BuzzerFreqSet(u8),
    BuzzerPlay(bool),
    ScreenPinSet { seg: usize, com: usize, value: bool },
    // Add more events as needed
}

type Callback = Rc<dyn Fn(&Event) + 'static>;

pub struct IOBus {
    subscribers: RefCell<Vec<Callback>>,
}

impl IOBus {
    pub fn new() -> Self {
        Self {
            subscribers: RefCell::new(Vec::new()),
        }
    }

    pub fn subscribe(&self, callback: Callback) {
        self.subscribers.borrow_mut().push(callback);
    }

    pub fn emit(&self, event: Event) {
        for cb in self.subscribers.borrow().iter() {
            cb(&event);
        }
    }
}
