use std::fmt;

#[repr(u8)]
pub enum LogLevel {
    Info = 0,
    Error = 1,
    Memory = 2,
    Cpu = 3,
    Interrupt = 4,
}

impl fmt::Display for LogLevel {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            LogLevel::Info => write!(f, "INFO"),
            LogLevel::Error => write!(f, "ERROR"),
            LogLevel::Memory => write!(f, "MEMORY"),
            LogLevel::Cpu => write!(f, "CPU"),
            LogLevel::Interrupt => write!(f, "INTERRUPT"),
        }
    }
}

pub trait Logger {
    fn log(&self, level: LogLevel, message: &str);
    fn log_enabled(&self, level: LogLevel) -> bool;
}