use tamalib_rs::{LogLevel, Logger};

pub struct TamaLogger {
    enabled: [bool; 5],
}

impl TamaLogger {
    pub fn new() -> Self {
        Self {
            enabled: [true, true, false, false, false],
        }
    }

    pub fn set_log_level(&mut self, level: LogLevel, enabled: bool) {
        self.enabled[level as usize] = enabled;
    }
}


impl Logger for TamaLogger {
    fn log(&self, level: LogLevel, message: &str) {
        println!("{}> {}", level, message);
    }

    fn log_enabled(&self, level: LogLevel) -> bool {
        self.enabled[level as usize]
    }
}