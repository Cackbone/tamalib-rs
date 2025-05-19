use wasm_bindgen::prelude::*;
use tamalib_rs::{Tamagotchi, Screen, Buzzer, Logger, LogLevel};
use std::rc::Rc;
use std::cell::RefCell;

use js_sys::Date;
use web_sys::console;


#[wasm_bindgen]
extern "C" {
    // Screen trait JS bindings
    #[wasm_bindgen(js_namespace = window)]
    fn js_screen_update();
    #[wasm_bindgen(js_namespace = window)]
    fn js_screen_set_pixel(x: usize, y: usize, value: bool);
    #[wasm_bindgen(js_namespace = window)]
    fn js_screen_set_icon(icon: usize, value: bool);
    // Buzzer trait JS bindings
    #[wasm_bindgen(js_namespace = window)]
    fn js_buzzer_set_frequency(freq: usize);
    #[wasm_bindgen(js_namespace = window)]
    fn js_buzzer_play(value: bool);
}

pub struct WasmScreen;
impl Screen for WasmScreen {
    fn update(&mut self) {
        js_screen_update();
    }
    fn set_pixel(&mut self, x: usize, y: usize, value: bool) {
        js_screen_set_pixel(x, y, value);
    }
    fn set_icon(&mut self, icon: usize, value: bool) {
        js_screen_set_icon(icon, value);
    }
}

pub struct WasmBuzzer;
impl Buzzer for WasmBuzzer {
    fn set_frequency(&mut self, freq: usize) {
        js_buzzer_set_frequency(freq);
    }
    fn play(&mut self, value: bool) {
        js_buzzer_play(value);
    }
}

#[cfg(target_arch = "wasm32")]
pub struct WasmClock;


#[cfg(target_arch = "wasm32")]
impl tamalib_rs::Clock for WasmClock {
    fn now(&self) -> usize {
        Date::now() as usize
    }
}

pub struct WasmLogger {
    cpu_enabled: bool,
    memory_enabled: bool,
}

impl WasmLogger {
    pub fn new() -> Self {
        Self {
            cpu_enabled: false,
            memory_enabled: false,
        }
    }

    fn set_log_level(&mut self, level: LogLevel, enabled: bool) {
        match level {
            LogLevel::Cpu => self.cpu_enabled = enabled,
            LogLevel::Memory => self.memory_enabled = enabled,
            _ => {}
        }
    }
}

impl Logger for WasmLogger {
    fn log(&self, level: LogLevel, msg: &str) {
        console::log_1(&format!("[{}] {}", level, msg).into()); 
    }

    fn log_enabled(&self, level: LogLevel) -> bool {
        match level {
            LogLevel::Cpu => self.cpu_enabled,
            LogLevel::Memory => self.memory_enabled,
            _ => true,
        }
    }
}

#[wasm_bindgen]
pub struct WasmTamagotchi {
    inner: Rc<RefCell<Tamagotchi>>,
}

#[wasm_bindgen]
impl WasmTamagotchi {
    #[wasm_bindgen(constructor)]
    pub fn new(rom: &[u8]) -> WasmTamagotchi {
        if rom.is_empty() {
            panic!("ROM cannot be empty!");
        }
        println!("WasmTamagotchi: new");
        let screen = Rc::new(RefCell::new(WasmScreen));
        let buzzer = Box::new(WasmBuzzer);
        let system_clock = Box::new(WasmClock);
        let logger = Box::new(WasmLogger::new());

        let tamagotchi = Tamagotchi::builder()
            .rom(rom.to_vec())
            .screen(screen)
            .buzzer(buzzer)
            .system_clock(system_clock)
            .logger(logger)
            .build();


        WasmTamagotchi {
            inner: Rc::new(RefCell::new(tamagotchi)),
        }
    }

    #[cfg(target_arch = "wasm32")]
    #[wasm_bindgen]
    pub fn run_step(&self) {
        self.inner.borrow_mut().run_step();
    }

    #[wasm_bindgen]
    pub fn set_button(&self, button: u8, pressed: bool) {
        use tamalib_rs::Button;
        let btn = match button {
            0 => Button::LEFT,
            1 => Button::MIDDLE,
            2 => Button::RIGHT,
            _ => return,
        };
        self.inner.borrow_mut().io.set_button(btn, pressed);
    }

    #[wasm_bindgen]
    pub fn set_speed(&self, speed: u8) {
        self.inner.borrow_mut().set_speed(speed);
    }
}

#[wasm_bindgen(start)]
pub fn main() {
    console_error_panic_hook::set_once();
}
