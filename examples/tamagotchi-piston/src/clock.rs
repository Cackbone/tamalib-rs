use std::time::{SystemTime, UNIX_EPOCH};
use tamalib_rs::Clock;
pub struct SystemClock;


impl Clock for SystemClock {
    fn now(&self) -> usize {
        SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_micros() as usize
    }
}