mod cpu;
mod io;
mod rom;
mod logger;
use crate::cpu::{Cpu, InputPin};
use crate::io::iobus::{Event, IOBus};
use std::cell::RefCell;
use std::collections::VecDeque;
use std::rc::Rc;


pub use io::{Button, Buzzer, IO, Screen, SCREEN_WIDTH, SCREEN_HEIGHT, ICONS_COUNT};
pub use cpu::Clock;
pub use logger::{LogLevel, Logger};


#[derive(PartialEq, Eq)]
pub enum ExecMode {
    Pause,
    Run,
    Step,
    Next,
    ToCall,
    ToReturn
}


#[derive(Default)]
pub struct TamagotchiBuilder {
    rom: Option<Vec<u8>>,
    screen: Option<Rc<RefCell<dyn Screen>>>,
    buzzer: Option<Box<dyn Buzzer>>,
    system_clock: Option<Box<dyn Clock>>,
    logger: Option<Box<dyn Logger>>,
}

impl TamagotchiBuilder {
    pub fn rom(mut self, rom: Vec<u8>) -> Self {
        self.rom = Some(rom);
        self
    }

    pub fn screen(mut self, screen: Rc<RefCell<dyn Screen>>) -> Self {
        self.screen = Some(screen);
        self
    }

    pub fn buzzer(mut self, buzzer: Box<dyn Buzzer>) -> Self {
        self.buzzer = Some(buzzer);
        self
    }

    pub fn system_clock(mut self, system_clock: Box<dyn Clock>) -> Self {
        self.system_clock = Some(system_clock);
        self
    }

    pub fn logger(mut self, logger: Box<dyn Logger>) -> Self {
        self.logger = Some(logger);
        self
    }

    pub fn build(self) -> Tamagotchi {
        let rom = self.rom.unwrap();
        let screen = self.screen.unwrap();
        let buzzer = self.buzzer.unwrap();
        let system_clock = self.system_clock.unwrap();
        let logger = self.logger.unwrap();
        Tamagotchi::new(rom, screen, buzzer, system_clock, logger)
    }
}





pub struct Tamagotchi {
    cpu: Cpu,
    pub io: IO,
    event_queue: Rc<RefCell<Vec<Event>>>,
    pub framerate: usize,
    exec_mode: ExecMode,
    depth: usize,
    ts_freq: usize,
    screen_ts: usize,
    logger: Rc<RefCell<Box<dyn Logger>>>,
}

impl Tamagotchi {

    pub fn new(rom_bytes: Vec<u8>, screen: Rc<RefCell<dyn Screen>>, buzzer: Box<dyn Buzzer>, system_clock: Box<dyn Clock>, logger: Box<dyn Logger>) -> Self {
        let bus = Rc::new(IOBus::new());
        let event_queue = Rc::new(RefCell::new(Vec::new()));
        let event_queue_clone = event_queue.clone();
        bus.subscribe(Rc::new(move |event| {
            event_queue_clone.borrow_mut().push(event.clone());
        }));
        let breakpoints: VecDeque<usize> = VecDeque::from([]);
        let rom = rom::load_rom(rom_bytes);
        let logger = Rc::new(RefCell::new(logger));
        let cpu = Cpu::new(rom, 1000000, bus.clone(), system_clock, logger.clone(), breakpoints);
        let io = IO::new(screen, buzzer, bus.clone());
        let screen_ts = cpu.clock.system_clock.now();
        Self {
            cpu,
            io,
            event_queue,
            framerate: 30,
            exec_mode: ExecMode::Run,
            depth: 0,
            ts_freq: 1000000, 
            screen_ts,
            logger: logger.clone(),
        }
    }

    pub fn builder() -> TamagotchiBuilder {
        TamagotchiBuilder::default()
    }

    pub fn run_step(&mut self) {
        self.step();
        let now = self.cpu.clock.system_clock.now();
        let elapsed = now - self.screen_ts;

        if elapsed as u128 >= (self.ts_freq / self.framerate) as u128 {
            self.screen_ts = now;
            self.io.screen.borrow_mut().update();
        }
    }

    pub async fn run_async(&mut self) {
        self.exec_mode = ExecMode::Run;
        loop {
            self.step();
            let now = self.cpu.clock.system_clock.now();
            let elapsed = now - self.screen_ts;
            if elapsed as u128 >= (self.ts_freq / self.framerate) as u128 {
                self.screen_ts = now;
                self.io.screen.borrow_mut().update();
            }
        }
    }

    pub fn set_mode(&mut self, mode: ExecMode) {
        self.exec_mode = mode;
        self.depth = self.cpu.call_depth;
        self.cpu.sync_ref_timestamp();
    }

    pub fn set_speed(&mut self, speed: u8) {
        self.cpu.set_speed(speed);
        self.logger.borrow().log(LogLevel::Info, &format!("Speed set to {}", speed));
    }

    pub fn step(&mut self) {
        if self.exec_mode == ExecMode::Pause {
            return;
        }
        self.process_events();

        if self.cpu.step() {
            self.exec_mode = ExecMode::Pause;
            self.depth = self.cpu.call_depth;
            return;
        }

        match self.exec_mode {
            ExecMode::Pause | ExecMode::Run => {}
            ExecMode::Step => {
                self.exec_mode = ExecMode::Pause;
            }
            ExecMode::Next => {
                if self.cpu.call_depth <= self.depth {
                    self.exec_mode = ExecMode::Pause;
                    self.depth = self.cpu.call_depth;
                }
            }
            ExecMode::ToCall => {
                if self.cpu.call_depth > self.depth {
                    self.exec_mode = ExecMode::Pause;
                    self.depth = self.cpu.call_depth;
                }
            }
            ExecMode::ToReturn => {
                if self.cpu.call_depth < self.depth {
                    self.exec_mode = ExecMode::Pause;
                    self.depth = self.cpu.call_depth;
                }
            }
        }
    }


    pub fn process_events(&mut self) {
        let mut queue = self.event_queue.borrow_mut();
        for event in queue.drain(..) {
            match event {
                Event::ButtonPressed { pin, value } => {
                    if let Some(pin_enum) = InputPin::from_u8(pin) {
                        self.cpu.set_pin(pin_enum, value);
                    }
                }
                Event::BuzzerFreqSet(freq) => {
                    self.io.set_buzzer_freq(freq);
                }
                Event::BuzzerPlay(enabled) => {
                    self.io.play_buzzer(enabled);
                }
                Event::ScreenPinSet { seg, com, value } => {
                    self.io.set_screen_pin(seg, com, value);
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;    struct DummyScreen;
    impl Screen for DummyScreen {
        fn update(&mut self) {}
        fn set_pixel(&mut self, _x: usize, _y: usize, _value: bool) {}
        fn set_icon(&mut self, _icon: usize, _value: bool) {}
    }

    struct DummyBuzzer;
    impl Buzzer for DummyBuzzer {
        fn set_frequency(&mut self, _freq: usize) {}
        fn play(&mut self, _value: bool) {}
    }

    struct DummyClock;
    impl Clock for DummyClock {
        fn now(&self) -> usize { 0 }
    }

    struct DummyLogger;
    impl Logger for DummyLogger {
        fn log(&self, _level: LogLevel, _message: &str) {}
        fn log_enabled(&self, _level: LogLevel) -> bool { true }
    }

    #[test]
    fn test_tamagotchi_construction() {
        let rom = vec![0; 128]; // Use a non-empty dummy ROM
        let screen = Rc::new(RefCell::new(DummyScreen));
        let buzzer = Box::new(DummyBuzzer);
        let clock = Box::new(DummyClock);
        let logger = Box::new(DummyLogger);
        let _tama = Tamagotchi::new(rom, screen, buzzer, clock, logger);
        // You can add more assertions or debug prints here
    }
}
