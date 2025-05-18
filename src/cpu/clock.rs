pub const TICK_FREQUENCY: usize = 32768; // Hz

// Oscillator frequencies
pub const OSC1_FREQUENCY: usize = TICK_FREQUENCY; // Hz
pub const OSC3_FREQUENCY: usize = 1000000; // Hz


pub trait Clock {
    fn now(&self) -> usize;
}

#[repr(usize)]
pub enum TimerType {
    Timer2Hz = 0,
    Timer4Hz = 1,
    Timer8Hz = 2,
    Timer16Hz = 3,
    Timer32Hz = 4,
    Timer64Hz = 5,
    Timer128Hz = 6,
    Timer256Hz = 7
}

/// Clock timer
pub struct Timer {
    // Timestamp in ticks
    pub ts: usize,
    pub period: usize
}

impl Timer {
    pub fn new(period: usize) -> Self {
        Self { ts: 0, period }
    }
}


/// Programmable timer
pub struct ProgTimer {
    pub enabled: bool,
    // Timestamp in ticks
    pub ts: usize,
    pub reload: u8,
    pub data: u8,
}

impl ProgTimer {
    pub const PERIOD: usize = TICK_FREQUENCY / 256;

    pub fn new() -> Self {
        Self { enabled: false, ts: 0, reload: 0, data: 0 }
    }
}

pub struct CpuClock {
    pub timers: [Timer; 8],
    pub prog_timer: ProgTimer,
    

    pub tick_counter: usize,
    pub ts_freq: usize,
    pub speed_ratio: u8,
    pub ref_ts: usize, // timestamp

    pub cpu_halted: bool,
    pub cpu_freq: usize, // hz
    pub scaled_cycle_accumulator: usize,

    pub system_clock: Box<dyn Clock>,

    pub previous_cycles: u8
}


impl CpuClock {
    pub fn new(cpu_freq: usize, ts_freq: usize, system_clock: Box<dyn Clock>) -> Self {
        let now = system_clock.now();
        Self {
            timers: [
                Timer::new(TICK_FREQUENCY / 2), // 2hz
                Timer::new(TICK_FREQUENCY / 4), // 4hz
                Timer::new(TICK_FREQUENCY / 8), // 8hz
                Timer::new(TICK_FREQUENCY / 16), // 16hz
                Timer::new(TICK_FREQUENCY / 32), // 32hz
                Timer::new(TICK_FREQUENCY / 64), // 64hz
                Timer::new(TICK_FREQUENCY / 128), // 128hz
                Timer::new(TICK_FREQUENCY / 256) // 256hz
            ],
            prog_timer: ProgTimer::new(),
            tick_counter: 0,
            ts_freq,
            speed_ratio: 1,
            ref_ts: now,
            cpu_halted: false,
            cpu_freq,
            scaled_cycle_accumulator: 0,
            system_clock,
            previous_cycles: 0,
        }
    }

    pub fn wait_timer(&mut self, timer_type: TimerType) -> bool {
        let timer = &mut self.timers[timer_type as usize];
        
        if (self.tick_counter - timer.ts) >= timer.period {
            loop {
                timer.ts += timer.period;
                if (self.tick_counter - timer.ts) < timer.period {
                    break;
                }
            }

            return true;
        }

        false
    }
}