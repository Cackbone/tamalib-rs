mod instructions;
mod flags;
mod clock;
mod interrupt;
mod io_register;
mod inputs;

use flags::{Flag, Flags};
use clock::{CpuClock, TimerType, ProgTimer};
use interrupt::{Interrupt, InterruptId};
use io_register::IORegister;
use inputs::Inputs;
use crate::{io::iobus::{Event, IOBus}, LogLevel, Logger};

use std::{cell::RefCell, rc::Rc};
use std::collections::VecDeque;
//use tamalib_macros::register_instructions;

pub use clock::Clock;
pub use inputs::InputPin;
pub use instructions::{InstructionWithArgs};


pub const MEM_SIZE: usize = 4096;
// pub const MEM_RAM_ADDR: usize = 0x000;
pub const MEM_RAM_SIZE: usize = 0x300;
pub const MEM_DISPLAY1_ADDR: usize = 0xE00;
pub const MEM_DISPLAY1_SIZE: usize = 0x066;
pub const MEM_DISPLAY2_ADDR: usize = 0xE80;
pub const MEM_DISPLAY2_SIZE: usize = 0x066;
pub const MEM_IO_ADDR: usize = 0xF00;
pub const MEM_IO_SIZE: usize = 0x080;


// Epson E0C6S46 CPU
pub struct Cpu {
    pub pc: usize,      // Program counter
    pub next_pc: usize, // Next program counter
    pub sp: u8,         // Stack pointer
    pub np: u8,         // New page/bank pointer

    pub a: u8,  // A register
    pub b: u8,  // B register
    pub x: u16, // X register
    pub y: u16, // Y register

    pub flags: Flags,

    pub memory: [u8; MEM_SIZE],

    pub clock: CpuClock,

    pub call_depth: usize,

    // Interrupts in priority order
    pub interrupts: [Interrupt; 6],

    pub inputs: Inputs,
    pub iobus: Rc<IOBus>,

    pub program: Vec<u16>,
    pub breakpoints: VecDeque<usize>,

    pub logger: Rc<RefCell<Box<dyn Logger>>>,
}

// #[register_instructions]
impl Cpu {
    pub fn new(program: Vec<u16>, freq: usize, iobus: Rc<IOBus>, system_clock: Box<dyn Clock>, logger: Rc<RefCell<Box<dyn Logger>>>, breakpoints: VecDeque<usize>) -> Self {
        let mut cpu = Self {
            pc: 0,
            next_pc: 0,
            sp: 0,
            np: 0,
            a: 0,
            b: 0,
            x: 0,
            y: 0,
            flags: Flags::default(),
            memory: [0; MEM_SIZE],
            clock: CpuClock::new(clock::OSC1_FREQUENCY, freq, system_clock),
            call_depth: 0,
            interrupts: [
                Interrupt::new(InterruptId::ProgTimer, 0x0c),
                Interrupt::new(InterruptId::Serial, 0x0a),
                Interrupt::new(InterruptId::K10K13, 0x08),
                Interrupt::new(InterruptId::K00K03, 0x06),
                Interrupt::new(InterruptId::Stopwatch, 0x04),
                Interrupt::new(InterruptId::ClockTimer, 0x02),
            ],
            inputs: Inputs::new(),
            iobus: iobus.clone(),
            program,
            breakpoints,
            logger,
        };

        cpu.reset();
        cpu.inputs.init();

        cpu
    }

    pub fn set_speed(&mut self, speed: u8) {
        self.clock.speed_ratio = speed;
    }

    /// Constructs a program counter (PC) value from the given bank, page, and step.
    pub const fn to_pc(bank: usize, page: u8, step: u8) -> usize {
        (step as usize & 0xFF) | ((page as usize & 0xF) << 8) | ((bank & 0x1) << 12)
    }

    // Program counter step
    #[allow(dead_code)]
    pub const fn pcs(&self) -> usize {
        self.pc & 0xff
    }

    // Four low order bits of PCS
    pub const fn pcsl(&self) -> usize {
        self.pc & 0xf
    }

    // Four high order bits of PCS
    pub const fn pcsh(&self) -> usize {
        (self.pc >> 4) & 0xf
    }

    // Program counter page
    pub const fn pcp(&self) -> usize {
        (self.pc >> 8) & 0xf
    }

    // Program counter bank
    pub const fn pcb(&self) -> usize {
        (self.pc >> 12) & 0x1
    }

    /// Constructs a new page pointer (NP) value from the given bank and page.
    pub const fn to_np(bank: u8, page: u8) -> u8 {
        (page & 0xF) | ((bank & 0x1) << 4)
    }

    // New bank pointer
    pub const fn nbp(&self) -> u8 {
        (self.np >> 4) & 0x1
    }

    // New page pointer
    pub const fn npp(&self) -> u8 {
        self.np & 0xf
    }

    // Low order eight bits of index register IX
    pub const fn xhl(&self) -> u16 {
        self.x & 0xff
    }

    // Low order four bits of XHL register
    pub const fn xl(&self) -> u16 {
        self.x & 0xf
    }

    // High order four bits of XHL register
    pub const fn xh(&self) -> u16 {
        (self.x >> 4) & 0xf
    }

    // High order eight bits of index register IX
    pub const fn xp(&self) -> u16 {
        (self.x >> 8) & 0xf
    }

    // Low order eight bits of index register IY
    pub const fn yhl(&self) -> u16 {
        self.y & 0xff
    }

    // Low order four bits of YHL register
    pub const fn yl(&self) -> u16 {
        self.y & 0xf
    }

    // High order four bits of YHL register
    pub const fn yh(&self) -> u16 {
        (self.y >> 4) & 0xf
    }

    // High order eight bits of index register IY
    pub const fn yp(&self) -> u16 {
        (self.y >> 8) & 0xf
    }

    // Low order four bits of stack pointer
    pub const fn spl(&self) -> u8 {
        self.sp & 0xf
    }

    // High order four bits of stack pointer
    pub const fn sph(&self) -> u8 {
        (self.sp >> 4) & 0xf
    }

    /*
       Two-bit register code
       r, q is two-bit immediate data; according to
       the contents of these bits, they indicate
       registers A, B, and MX and MY (data
       memory whose addresses are specified with
       index registers IX and IY)

       +----+----+----+----+---------------------+
       | r1 | r0 | q1 | q0 | Register specified  |
       +----+----+----+----+---------------------+
       |  0 |  0 |  0 |  0 |         A           |
       |  0 |  1 |  0 |  1 |         B           |
       |  1 |  0 |  1 |  0 |        MX           |
       |  1 |  1 |  1 |  1 |        MY           |
       +----+----+----+----+---------------------+
    */
    pub fn set_rq(&mut self, rq: u16, value: u8) {
        match rq & 0x3 {
            0x0 => self.a = value,
            0x1 => self.b = value,
            0x2 => self.set_mem(self.x as usize, value),
            0x3 => self.set_mem(self.y as usize, value),
            _ => unreachable!(),
        }
    }

    pub fn get_rq(&mut self, rq: u16) -> u8 {
        match rq & 0x3 {
            0x0 => self.a,
            0x1 => self.b,
            0x2 => self.get_mem(self.x as usize),
            0x3 => self.get_mem(self.y as usize),
            _ => 0,
        }
    }

    pub fn set_mem(&mut self, addr: usize, value: u8) {
        if addr < MEM_RAM_SIZE {
            self.memory[addr] = value;
        } else if (addr >= MEM_DISPLAY1_ADDR && addr < MEM_DISPLAY1_ADDR + MEM_DISPLAY1_SIZE)
            || (addr >= MEM_DISPLAY2_ADDR && addr < MEM_DISPLAY2_ADDR + MEM_DISPLAY2_SIZE)
        {
            self.memory[addr] = value;
            self.set_lcd(addr, value);
        } else if addr >= MEM_IO_ADDR && addr < MEM_IO_ADDR + MEM_IO_SIZE {
            self.memory[addr] = value;
            self.set_io(IORegister::from_usize(addr), value);
        } else {
            if self.logger.borrow().log_enabled(LogLevel::Error) {
                self.logger.borrow().log(LogLevel::Error, &format!("WRITING OUT OF MEMORY AT: 0x{:03X}", addr));
            }
        }
        if self.logger.borrow().log_enabled(LogLevel::Memory) {
            self.logger.borrow().log(LogLevel::Memory, &format!("WRITE 0x{:02X} AT: 0x{:03X}", value, addr));
        }
    }

    pub fn get_mem(&mut self, addr: usize) -> u8 {
        if self.logger.borrow().log_enabled(LogLevel::Memory) {
            self.logger.borrow().log(LogLevel::Memory, &format!("READ 0x{:02X} FROM: 0x{:03X}", self.memory[addr], addr));
        }
        
        if (addr < MEM_RAM_SIZE)
            || (addr >= MEM_DISPLAY1_ADDR && addr < MEM_DISPLAY1_ADDR + MEM_DISPLAY1_SIZE)
            || (addr >= MEM_DISPLAY2_ADDR && addr < MEM_DISPLAY2_ADDR + MEM_DISPLAY2_SIZE)
        {
            return self.memory[addr];
        } else if addr >= MEM_IO_ADDR && addr < (MEM_IO_ADDR + MEM_IO_SIZE) {
            return self.get_io(IORegister::from_usize(addr));
        } else {
            if self.logger.borrow().log_enabled(LogLevel::Error) {
                self.logger.borrow().log(LogLevel::Error, &format!("READING OUT OF MEMORY AT: 0x{:03X}", addr));
            }
        }
    
        0
    }

    pub fn set_io(&mut self, reg: IORegister, value: u8) {
        match reg {
            IORegister::ClockIntMasks => {
                self.interrupts[InterruptId::ClockTimer as usize].mask = value;
            }
            IORegister::SwIntMasks => {
                self.interrupts[InterruptId::Stopwatch as usize].mask = value;
            }
            IORegister::ProgIntMasks => {
                self.interrupts[InterruptId::ProgTimer as usize].mask = value;
            }
            IORegister::SerialIntMasks => {
                self.interrupts[InterruptId::K10K13 as usize].mask = value;
            }
            IORegister::K00K03IntMasks => {
                self.interrupts[InterruptId::K00K03 as usize].mask = value;
            }
            IORegister::K10K13IntMasks => {
                self.interrupts[InterruptId::K10K13 as usize].mask = value;
            }
            IORegister::ClockTimerData1 => { /* Read only */ }
            IORegister::ClockTimerData2 => { /* Read only */ }
            IORegister::ProgTimerReloadDataL => {
                self.clock.prog_timer.reload = value | (self.clock.prog_timer.reload & 0xf0);
            }
            IORegister::ProgTimerReloadDataH => {
                self.clock.prog_timer.reload = (self.clock.prog_timer.reload & 0xF) | (value << 4);
            }
            IORegister::K00K03InputPort => { /* Read only */ }
            IORegister::K00K03InputRelation => { /* Read only */ }
            IORegister::R40R43BzOutputPort => {
                // Emit buzzer play event
                self.iobus.emit(Event::BuzzerPlay(value & 0x8 == 0));
            }
            IORegister::CpuOsc3Ctrl => {
                /* CPU (osc1)/osc3 clocks switch */
                if (value & 0x8) != 0 && !(self.clock.cpu_freq == clock::OSC3_FREQUENCY) {
                    self.clock.cpu_freq = clock::OSC3_FREQUENCY;
                    self.clock.scaled_cycle_accumulator = 0;
                } else if (value & 0x8) == 0 && !(self.clock.cpu_freq == clock::OSC1_FREQUENCY) {
                    self.clock.cpu_freq = clock::OSC1_FREQUENCY;
                    self.clock.scaled_cycle_accumulator = 0;
                }
            }
            IORegister::LcdCtrl => { /* Lcd control nothing to do */ }
            IORegister::LcdContrast => { /* Lcd contrast, assume medium contrast (0x8) */ }
            IORegister::SvdCtrl => { /* SVD control, assume battery voltage always ok (0x6) */ }
            IORegister::BuzzerCtrl1 => {
                // Emit buzzer frequency event
                self.iobus.emit(Event::BuzzerFreqSet(value & 0x7));
            }
            IORegister::BuzzerCtrl2 => { /* Buzzer control nothing to do */ }
            IORegister::ClkWdTimerCtrl => { /* Clock watchdog timer, ignored */ }
            IORegister::SwTimerCtrl => { /* Stopwatch timer control nothing to do */ }
            IORegister::ProgTimerClkSel => { /* Programmable timer selection, assume 256hz, output disabled */
            }
            IORegister::ProgTimerCtrl => {
                if (value & 0x2) != 0 {
                    self.clock.prog_timer.data = self.clock.prog_timer.reload;
                }
                if ((value & 0x1) != 0) && !(self.clock.prog_timer.enabled) {
                    self.clock.prog_timer.ts = self.clock.tick_counter;
                }

                self.clock.prog_timer.enabled = (value & 0x01) != 0;
            }
            _ => {
                if self.logger.borrow().log_enabled(LogLevel::Error) {
                    self.logger.borrow().log(LogLevel::Error, &format!("WRITING TO UNKNOWN IO REGISTER: 0x{:03X}", reg as usize));
                }
            }
        }
    }

    pub fn get_io(&mut self, reg: IORegister) -> u8 {
        match reg {
            IORegister::ClkIntFactorFlags => {
                let i = self.interrupts[InterruptId::ClockTimer as usize].factor;
                self.interrupts[InterruptId::ClockTimer as usize].factor = 0;
                i
            }
            IORegister::SwIntFactorFlags => {
                let i = self.interrupts[InterruptId::Stopwatch as usize].factor;
                self.interrupts[InterruptId::Stopwatch as usize].factor = 0;
                i
            }
            IORegister::ProgIntFactorFlags => {
                let i = self.interrupts[InterruptId::ProgTimer as usize].factor;
                self.interrupts[InterruptId::ProgTimer as usize].factor = 0;
                i
            }
            IORegister::SerialIntFactorFlags => {
                let i = self.interrupts[InterruptId::Serial as usize].factor;
                self.interrupts[InterruptId::Serial as usize].factor = 0;
                i
            }
            IORegister::K00K03IntFactorFlags => {
                let i = self.interrupts[InterruptId::K00K03 as usize].factor;
                self.interrupts[InterruptId::K00K03 as usize].factor = 0;
                i
            }
            IORegister::K10K13IntFactorFlags => {
                let i = self.interrupts[InterruptId::K10K13 as usize].factor;
                self.interrupts[InterruptId::K10K13 as usize].factor = 0;
                i
            }
            IORegister::ClockIntMasks => self.interrupts[InterruptId::ClockTimer as usize].mask,
            IORegister::SwIntMasks => self.interrupts[InterruptId::Stopwatch as usize].mask & 0x3,
            IORegister::ProgIntMasks => self.interrupts[InterruptId::ProgTimer as usize].mask & 0x1,
            IORegister::SerialIntMasks => self.interrupts[InterruptId::Serial as usize].mask & 0x1,
            IORegister::K00K03IntMasks => self.interrupts[InterruptId::K00K03 as usize].mask,
            IORegister::K10K13IntMasks => self.interrupts[InterruptId::K10K13 as usize].mask,
            IORegister::ClockTimerData1 => self.memory[reg as usize],
            IORegister::ClockTimerData2 => self.memory[reg as usize],
            IORegister::ProgTimerDataL => self.clock.prog_timer.data & 0xf,
            IORegister::ProgTimerDataH => (self.clock.prog_timer.data >> 4) & 0xf,
            IORegister::ProgTimerReloadDataL => self.clock.prog_timer.reload & 0xf,
            IORegister::ProgTimerReloadDataH => (self.clock.prog_timer.reload >> 4) & 0xf,
            IORegister::K00K03InputPort => self.inputs.ports[0],
            IORegister::K00K03InputRelation => self.memory[reg as usize],
            IORegister::K10K13InputPort => self.inputs.ports[1],
            IORegister::R40R43BzOutputPort => self.memory[reg as usize],
            IORegister::CpuOsc3Ctrl => self.memory[reg as usize],
            IORegister::LcdCtrl => self.memory[reg as usize],
            IORegister::LcdContrast => 0,
            IORegister::SvdCtrl => self.memory[reg as usize] & 0x7,
            IORegister::BuzzerCtrl1 => self.memory[reg as usize] & 0x7,
            IORegister::BuzzerCtrl2 => self.memory[reg as usize] & 0x7,
            IORegister::ClkWdTimerCtrl => 0,
            IORegister::SwTimerCtrl => 0,
            IORegister::ProgTimerCtrl => self.clock.prog_timer.enabled as u8,
            IORegister::ProgTimerClkSel => 0,
            _ => {
                if self.logger.borrow().log_enabled(LogLevel::Error) {
                    self.logger.borrow().log(LogLevel::Error, &format!("READING FROM UNKNOWN IO REGISTER: 0x{:03X}", reg as usize));
                }
                0
            }
        }
    }

    fn generate_interrupt(&mut self, id: InterruptId, bit: u8) {
        self.interrupts[id as usize].factor |= 0x1 << bit;

        if (self.interrupts[id as usize].mask & (0x1 << bit)) != 0 {
            self.interrupts[id as usize].triggered = true;
        }
    }

    pub fn set_pin(&mut self, pin: InputPin, value: bool) {
        let old_state = self.inputs.get_pin(pin);

        if value != old_state {
            match (pin as u8 & 0x4) >> 2 {
                0 => {
                    if value != ((self.memory[IORegister::K00K03InputRelation as usize] >> (pin as usize & 0x3) & 0x1) != 0)
                    {
                        self.generate_interrupt(InterruptId::K00K03, pin as u8 & 0x3);
                    }
                }
                1 => {
                    if value == false {
                        self.generate_interrupt(InterruptId::K10K13, pin as u8 & 0x3);
                    }
                }
                _ => {}
            }
        }

        self.inputs.set_pin(pin, value);
    }

    pub fn set_lcd(&mut self, n: usize, value: u8) {
        let seg = (n & 0x7f) >> 1;
        let com0 = ((n & 0x80) >> 7) * 8 + (n & 0x01) * 4;

        for i in 0..4 {
            let pin_value = ((value >> i) & 0x1) != 0;
            self.iobus.emit(Event::ScreenPinSet {
                seg,
                com: com0 + i,
                value: pin_value,
            });
        }
    }

    #[allow(dead_code)]
    pub fn refresh_io(&mut self) {
        const REFRESH_LOCS: [(usize, usize); 4] = [
            (MEM_DISPLAY1_ADDR, MEM_DISPLAY1_SIZE),
            (MEM_DISPLAY2_ADDR, MEM_DISPLAY2_SIZE),
            (IORegister::BuzzerCtrl1 as usize, 1),
            (IORegister::R40R43BzOutputPort as usize, 1),
        ];

        for (addr, size) in REFRESH_LOCS {
            for n in addr..(addr + size) {
                let mem = self.get_mem(n);
                self.set_mem(n, mem);
            }
        }
    }

    pub fn sync_ref_timestamp(&mut self) {
        let now = self.clock.system_clock.now();
        self.clock.ref_ts = now;
    }

    pub fn reset(&mut self) {
        self.pc = Self::to_pc(0, 1, 0x00);
        self.np = Self::to_np(0, 1);
        self.a = 0;
        self.b = 0;
        self.x = 0;
        self.y = 0;
        self.sp = 0;
        self.flags = Flags(0);

        self.memory.fill(0);

        self.memory[IORegister::R40R43BzOutputPort as usize] = 0xF;
        self.memory[IORegister::LcdCtrl as usize] = 0x8;
        self.memory[IORegister::K00K03InputRelation as usize] = 0xF;

        self.clock.cpu_freq = clock::OSC1_FREQUENCY;
        self.sync_ref_timestamp();
    }

    pub fn handle_timers(&mut self) {
        // 2Hz
        if self.clock.wait_timer(TimerType::Timer2Hz) {
            self.memory[IORegister::ClockTimerData2 as usize] ^= 0x1 << 3;
            if ((self.memory[IORegister::ClockTimerData2 as usize] >> 3) & 0x1) == 0 {
                self.generate_interrupt(InterruptId::ClockTimer, 3);
            }
        }
        // 4Hz
        if self.clock.wait_timer(TimerType::Timer4Hz) {
            self.memory[IORegister::ClockTimerData2 as usize] ^= 0x1 << 2;
            if ((self.memory[IORegister::ClockTimerData2 as usize] >> 2) & 0x1) == 0 {
                self.generate_interrupt(InterruptId::ClockTimer, 2);
            }
        }
        // 8Hz
        if self.clock.wait_timer(TimerType::Timer8Hz) {
            self.memory[IORegister::ClockTimerData2 as usize] ^= 0x1 << 1;
        }
        // 16Hz
        if self.clock.wait_timer(TimerType::Timer16Hz) {
            self.memory[IORegister::ClockTimerData1 as usize] ^= 0x1 << 0;
            if ((self.memory[IORegister::ClockTimerData1 as usize] >> 0) & 0x1) == 0 {
                self.generate_interrupt(InterruptId::ClockTimer, 1);
            }
        }
        // 32Hz
        if self.clock.wait_timer(TimerType::Timer32Hz) {
            self.memory[IORegister::ClockTimerData1 as usize] ^= 0x1 << 3;
        }
        // 64Hz
        if self.clock.wait_timer(TimerType::Timer64Hz) {
            self.memory[IORegister::ClockTimerData1 as usize] ^= 0x1 << 2;
            if ((self.memory[IORegister::ClockTimerData1 as usize] >> 2) & 0x1) == 0 {
                self.generate_interrupt(InterruptId::ClockTimer, 0);
            }
        }
        // 128Hz
        if self.clock.wait_timer(TimerType::Timer128Hz) {
            self.memory[IORegister::ClockTimerData1 as usize] ^= 0x1 << 1;
        }
        // 256Hz
        if self.clock.wait_timer(TimerType::Timer256Hz) {
            self.memory[IORegister::ClockTimerData1 as usize] ^= 0x1 << 0;
        }
        // Prog timer
        if self.clock.prog_timer.enabled
            && ((self.clock.tick_counter - self.clock.prog_timer.ts) >= ProgTimer::PERIOD)
        {
            loop {
                self.clock.prog_timer.ts += ProgTimer::PERIOD;
                self.clock.prog_timer.data -= 1;

                if self.clock.prog_timer.data == 0 {
                    self.clock.prog_timer.data = self.clock.prog_timer.reload;
                    self.generate_interrupt(InterruptId::ProgTimer, 0);
                }

                if (self.clock.tick_counter - self.clock.prog_timer.ts) < ProgTimer::PERIOD {
                    break;
                }
            }
        }
    }

    pub fn wait_for_cycles(&mut self, since: usize, cycles: u8) -> usize {
        self.clock.scaled_cycle_accumulator += cycles as usize * clock::TICK_FREQUENCY;
        let ticks_pending = self.clock.scaled_cycle_accumulator / self.clock.cpu_freq;

        if ticks_pending > 0 {
            self.clock.tick_counter += ticks_pending;
            self.clock.scaled_cycle_accumulator -= ticks_pending * self.clock.cpu_freq;
        }

        // Emulate as fast as possible
        if self.clock.speed_ratio == 0 {
            return self.clock.system_clock.now();
        }

        let nanos = ((cycles as usize) * self.clock.ts_freq)
            / (self.clock.cpu_freq * self.clock.speed_ratio as usize);
        let deadline = since + nanos;

        loop {
            if self.clock.system_clock.now() >= deadline {
                break;
            }
        }

        deadline
    }

    pub fn process_interrupts(&mut self) {
        for i in 0..6 {
            if self.interrupts[i].triggered {
                self.set_mem(((self.sp - 1) & 0xff) as usize, self.pcp() as u8);
                self.set_mem(((self.sp - 2) & 0xff) as usize, self.pcsh() as u8);
                self.set_mem(((self.sp - 3) & 0xff) as usize, self.pcsl() as u8);

                self.sp = (self.sp - 3) & 0xff;

                self.flags.set(Flag::I, false);

                self.np = Self::to_np(self.nbp(), 1);
                self.pc = Self::to_pc(self.pcb(), 1, self.interrupts[i].vector);
                self.call_depth += 1;
                self.clock.ref_ts = self.wait_for_cycles(self.clock.ref_ts, 12);
                self.interrupts[i].triggered = false;
                return;
            }
        }
    }

    pub fn print_state(&self, instr: &InstructionWithArgs) {
        if !self.logger.borrow().log_enabled(LogLevel::Cpu) {
            return;
        }

        let mut state = format!("0x{:04X}: ", self.pc);

        if self.call_depth < 100 {
            state.push_str(&" ".repeat(self.call_depth as usize));
        } else {
            state.push_str("...");
        }

        state.push_str(&format!("{:<21}", instr.to_string()));

        if self.call_depth < 10 {
            state.push_str(&" ".repeat(10 - self.call_depth as usize));
        }

        self.logger.borrow().log(LogLevel::Cpu, &format!("{} | SP=0x{:02X}, NP=0x{:02X}, X=0x{:03X}, Y=0x{:03X}, A=0x{:X}, B=0x{:X}, F=0x{:X}", state, self.sp, self.np, self.x, self.y, self.a, self.b, self.flags.0));
    }

    /// Step the CPU one instruction.
    pub fn step(&mut self) -> bool {
        let mut bp = self.breakpoints.pop_front();
        let mut pset_or_ei = false;

        if !self.clock.cpu_halted {
            let op = self.program[self.pc];

            if let Some(instr) = InstructionWithArgs::from_code(op) {
                self.next_pc = (self.pc + 1) & 0x1fff;
                self.clock.ref_ts = self.wait_for_cycles(self.clock.ref_ts, self.clock.previous_cycles);

                self.print_state(&instr);
                instr.exec(self);
                

                // Prepare for next instruction
                self.pc = self.next_pc;
                self.clock.previous_cycles = instr.get_cycles();

                let is_pset = instr.is_pset();
                pset_or_ei = is_pset || instr.is_ei();

                // Reset NP
                if !is_pset {
                    self.np = ((self.pc >> 8) & 0x1f) as u8;
                }
            } else {
                return true;
            }

        } else {
            self.clock.ref_ts = self.wait_for_cycles(self.clock.ref_ts, 5);
            self.clock.previous_cycles = 0;
        }

        self.handle_timers();

        if self.flags.get(Flag::I) && !pset_or_ei {
            self.process_interrupts();
        }

        while !self.clock.cpu_halted && bp.is_some() {
            if bp.unwrap() == self.pc {
                return true;
            }

            bp = self.breakpoints.pop_front();
        }

        return false;
    }

    // Instructions
    /// PSET: Set new page pointer (NP) to arg1.
    // #[instruction("PSET #0x{:02X}            ", 0xE40, MASK_7B, 0, 0, 5)]
    pub fn pset_cb(&mut self, arg1: u8, _arg2: u8) {
        self.np = arg1;
    }
    /// JP: Jump to address arg1 in current page.
    // #[instruction("JP   #0x{:02X}            ", 0x000, MASK_4B, 0, 0, 5)]
    pub fn jp_cb(&mut self, arg1: u8, _arg2: u8) {
        self.next_pc = arg1 as usize | ((self.np as usize) << 8);
    }
    /// JP C: Jump to address arg1 if carry flag is set.
    // #[instruction("JP   C #0x{:02X}          ", 0x200, MASK_4B, 0, 0, 5)]
    pub fn jp_c_cb(&mut self, arg1: u8, _arg2: u8) {
        if self.flags.get(Flag::C) {
            self.next_pc = arg1 as usize | ((self.np as usize) << 8);
        }
    }
    /// JP NC: Jump to address arg1 if carry flag is not set.
    // #[instruction("JP   NC #0x{:02X}         ", 0x300, MASK_4B, 0, 0, 5)]
    pub fn jp_nc_cb(&mut self, arg1: u8, _arg2: u8) {
        if !self.flags.get(Flag::C) {
            self.next_pc = arg1 as usize | ((self.np as usize) << 8);
        }
    }
    /// JP Z: Jump to address arg1 if zero flag is set.
    // #[instruction("JP   Z #0x{:02X}          ", 0x600, MASK_4B, 0, 0, 5)]
    pub fn jp_z_cb(&mut self, arg1: u8, _arg2: u8) {
        if self.flags.get(Flag::Z) {
            self.next_pc = arg1 as usize | ((self.np as usize) << 8);
        }
    }
    /// JP NZ: Jump to address arg1 if zero flag is not set.
    // #[instruction("JP   NZ #0x{:02X}         ", 0x700, MASK_4B, 0, 0, 5)]
    pub fn jp_nz_cb(&mut self, arg1: u8, _arg2: u8) {
        if !self.flags.get(Flag::Z) {
            self.next_pc = arg1 as usize | ((self.np as usize) << 8);
        }
    }
    /// JPBA: Jump to address formed from A, B, and NP registers.
    // #[instruction("JPBA                  ", 0xFE8, MASK_12B, 0, 0, 5)]
    pub fn jpba_cb(&mut self, _: u8, _: u8) {
        self.next_pc = self.a as usize | ((self.b as usize) << 4) | ((self.np as usize) << 8);
    }
    /// CALL: Call subroutine at address arg1 in current page.
    // #[instruction("CALL #0x{:02X}            ", 0x400, MASK_4B, 0, 0, 7)]
    pub fn call_cb(&mut self, arg1: u8, _arg2: u8) {
        self.pc = (self.pc + 1) & 0x1fff;
        self.set_mem(self.sp.wrapping_sub(1) as usize, self.pcp() as u8);
        self.set_mem(self.sp.wrapping_sub(2) as usize, self.pcsh() as u8);
        self.set_mem(self.sp.wrapping_sub(3) as usize, self.pcsl() as u8);

        self.sp = self.sp.wrapping_sub(3);

        self.next_pc = Self::to_pc(self.pcb(), self.npp(), arg1);
        self.call_depth += 1;
    }
    /// CALZ: Call subroutine at address arg1 in page 0.
    // #[instruction("CALZ #0x{:02X}            ", 0x500, MASK_4B, 0, 0, 7)]
    pub fn calz_cb(&mut self, arg1: u8, _arg2: u8) {
        self.pc = (self.pc + 1) & 0x1fff;
        self.set_mem(self.sp.wrapping_sub(1) as usize, self.pcp() as u8);
        self.set_mem(self.sp.wrapping_sub(2) as usize, self.pcsh() as u8);
        self.set_mem(self.sp.wrapping_sub(3) as usize, self.pcsl() as u8);

        self.sp = self.sp.wrapping_sub(3);

        self.next_pc = Self::to_pc(self.pcb(), 0, arg1);
        self.call_depth += 1;
    }
    /// RET: Return from subroutine.
    // #[instruction("RET                   ", 0xFDF, MASK_12B, 0, 0, 7)]
    pub fn ret_cb(&mut self, _arg1: u8, _arg2: u8) {
        self.next_pc = (self.get_mem(self.sp as usize) as usize)
            | ((self.get_mem(self.sp.wrapping_add(1) as usize) as usize) << 4)
            | ((self.get_mem(self.sp.wrapping_add(2) as usize) as usize) << 8)
            | ((self.pcb() as usize) << 12);
        self.sp = self.sp.wrapping_add(3);
        if self.call_depth > 0 {
            self.call_depth -= 1;
        }
    }
    /// RETS: Return from subroutine and increment PC.
    // #[instruction("RETS                  ", 0xFDE, MASK_12B, 0, 0, 12)]
    pub fn rets_cb(&mut self, _arg1: u8, _arg2: u8) {
        self.next_pc = (self.get_mem(self.sp as usize) as usize)
            | ((self.get_mem(self.sp.wrapping_add(1) as usize) as usize) << 4)
            | ((self.get_mem(self.sp.wrapping_add(2) as usize) as usize) << 8)
            | ((self.pcb() as usize) << 12);
        self.sp = self.sp.wrapping_add(3);
        self.next_pc = (self.next_pc + 1) & 0x1fff;

        if self.call_depth > 0 {
            self.call_depth -= 1;
        }
    }
    /// RETD: Return from subroutine, store arg1 in memory, and increment X.
    // #[instruction("RETD #0x{:02X}            ", 0x100, MASK_4B, 0, 0, 12)]
    pub fn retd_cb(&mut self, arg1: u8, _arg2: u8) {
        self.next_pc = (self.get_mem(self.sp as usize) as usize)
            | ((self.get_mem(self.sp.wrapping_add(1) as usize) as usize) << 4)
            | ((self.get_mem(self.sp.wrapping_add(2) as usize) as usize) << 8)
            | ((self.pcb() as usize) << 12);
        self.sp = self.sp.wrapping_add(3);
        self.set_mem(self.x as usize, arg1 & 0xf);
        self.set_mem(
            (((self.x.wrapping_add(1)) & 0xff) as usize) | ((self.xp() as usize) << 8),
            (arg1 >> 4) & 0xf,
        );
        self.x = ((self.x.wrapping_add(2)) & 0xff) | (self.xp() << 8);

        if self.call_depth > 0 {
            self.call_depth -= 1;
        }
    }
    /// NOP5: No operation (5 cycles).
    // #[instruction("NOP5                  ", 0xFFB, MASK_12B, 0, 0, 5)]
    pub fn nop5_cb(&mut self, _arg1: u8, _arg2: u8) {}
    /// NOP7: No operation (7 cycles).
    // #[instruction("NOP7                  ", 0xFFF, MASK_12B, 0, 0, 7)]
    pub fn nop7_cb(&mut self, _arg1: u8, _arg2: u8) {}
    /// HALT: Halt the CPU.
    // #[instruction("HALT                  ", 0xFF8, MASK_12B, 0, 0, 5)]
    pub fn halt_cb(&mut self, _arg1: u8, _arg2: u8) {
        self.clock.cpu_halted = true;
    }
    /// INC X: Increment X register.
    // #[instruction("INC  X #0x{:02X}          ", 0xEE0, MASK_12B, 0, 0, 5)]
    pub fn inc_x_cb(&mut self, _arg1: u8, _arg2: u8) {
        self.x = ((self.x + 1) & 0xff) | (self.xp() << 8);
    }
    /// INC Y: Increment Y register.
    // #[instruction("INC  Y #0x{:02X}          ", 0xEF0, MASK_12B, 0, 0, 5)]
    pub fn inc_y_cb(&mut self, _arg1: u8, _arg2: u8) {
        self.y = ((self.y + 1) & 0xff) | (self.yp() << 8);
    }
    /// LD X: Load arg1 into X register (low byte).
    // #[instruction("LD   X #0x{:02X}          ", 0xB00, MASK_4B, 0, 0, 5)]
    pub fn ld_x_cb(&mut self, arg1: u8, _arg2: u8) {
        self.x = arg1 as u16 | (self.xp() << 8);
    }
    /// LD Y: Load arg1 into Y register (low byte).
    // #[instruction("LD   Y #0x{:02X}          ", 0x800, MASK_4B, 0, 0, 5)]
    pub fn ld_y_cb(&mut self, arg1: u8, _arg2: u8) {
        self.y = arg1 as u16 | (self.yp() << 8);
    }
    /// LD XP R: Load XP from register specified by arg1.
    // #[instruction("LD   XP R({:X})          ", 0xE80, MASK_10B, 0, 0, 5)]
    pub fn ld_xp_r_cb(&mut self, arg1: u8, _arg2: u8) {
        self.x = self.xhl() | ((self.get_rq(arg1 as u16) as u16) << 8);
    }
    /// LD XH R: Load XH from register specified by arg1.
    // #[instruction("LD   XH R({:X})          ", 0xE84, MASK_10B, 0, 0, 5)]
    pub fn ld_xh_r_cb(&mut self, arg1: u8, _arg2: u8) {
        self.x = self.xl() | (self.get_rq(arg1 as u16) << 4) as u16 | (self.xp() << 8);
    }
    /// LD XL R: Load XL from register specified by arg1.
    // #[instruction("LD   XL R({:X})          ", 0xE88, MASK_10B, 0, 0, 5)]
    pub fn ld_xl_r_cb(&mut self, arg1: u8, _arg2: u8) {
        self.x = self.get_rq(arg1 as u16) as u16 | (self.xh() << 4) | (self.xp() << 8);
    }
    /// LD YP R: Load YP from register specified by arg1.
    // #[instruction("LD   YP R({:X})          ", 0xE90, MASK_10B, 0, 0, 5)]
    pub fn ld_yp_r_cb(&mut self, arg1: u8, _arg2: u8) {
        self.y = self.yhl() | ((self.get_rq(arg1 as u16) as u16) << 8);
    }
    /// LD YH R: Load YH from register specified by arg1.
    // #[instruction("LD   YH R({:X})          ", 0xE94, MASK_10B, 0, 0, 5)]
    pub fn ld_yh_r_cb(&mut self, arg1: u8, _arg2: u8) {
        self.y = self.yl() | (self.get_rq(arg1 as u16) << 4) as u16 | (self.yp() << 8);
    }
    /// LD YL R: Load YL from register specified by arg1.
    // #[instruction("LD   YL R({:X})          ", 0xE98, MASK_10B, 0, 0, 5)]
    pub fn ld_yl_r_cb(&mut self, arg1: u8, _arg2: u8) {
        self.y = self.get_rq(arg1 as u16) as u16 | (self.yh() << 4) | (self.yp() << 8);
    }
    /// LD R XP: Store XP into register specified by arg1.
    // #[instruction("LD   R({:X}) XP          ", 0xEA0, MASK_10B, 0, 0, 5)]
    pub fn ld_r_xp_cb(&mut self, arg1: u8, _arg2: u8) {
        self.set_rq(arg1 as u16, self.xp() as u8);
    }
    /// LD R XH: Store XH into register specified by arg1.
    // #[instruction("LD   R({:X}) XH          ", 0xEA4, MASK_10B, 0, 0, 5)]
    pub fn ld_r_xh_cb(&mut self, arg1: u8, _arg2: u8) {
        self.set_rq(arg1 as u16, self.xh() as u8);
    }
    /// LD R XL: Store XL into register specified by arg1.
    // #[instruction("LD   R({:X}) XL          ", 0xEA8, MASK_10B, 0, 0, 5)]
    pub fn ld_r_xl_cb(&mut self, arg1: u8, _arg2: u8) {
        self.set_rq(arg1 as u16, self.xl() as u8);
    }
    /// LD R YP: Store YP into register specified by arg1.
    // #[instruction("LD   R({:X}) YP          ", 0xEB0, MASK_10B, 0, 0, 5)]
    pub fn ld_r_yp_cb(&mut self, arg1: u8, _arg2: u8) {
        self.set_rq(arg1 as u16, self.yp() as u8);
    }
    /// LD R YH: Store YH into register specified by arg1.
    // #[instruction("LD   R({:X}) YH          ", 0xEB4, MASK_10B, 0, 0, 5)]
    pub fn ld_r_yh_cb(&mut self, arg1: u8, _arg2: u8) {
        self.set_rq(arg1 as u16, self.yh() as u8);
    }
    /// LD R YL: Store YL into register specified by arg1.
    // #[instruction("LD   R({:X}) YL          ", 0xEB8, MASK_10B, 0, 0, 5)]
    pub fn ld_r_yl_cb(&mut self, arg1: u8, _arg2: u8) {
        self.set_rq(arg1 as u16, self.yl() as u8);
    }
    /// ADC XH: Add arg1 and carry to XH.
    // #[instruction("ADC  XH #0x{:02X}         ", 0xA00, MASK_8B, 0, 0, 7)]
    pub fn adc_xh_cb(&mut self, arg1: u8, _arg2: u8) {
        let tmp = self.xh() as u16 + arg1 as u16 + (self.flags.get(Flag::C) as u8) as u16;
        self.x = self.xl() | (((tmp & 0xf) as u16) << 4) | (self.xp() << 8);
        self.flags.set(Flag::C, (tmp >> 4) != 0);
        self.flags.set(Flag::Z, (tmp & 0xf) == 0);
    }
    /// ADC XL: Add arg1 and carry to XL.
    // #[instruction("ADC  XL #0x{:02X}         ", 0xA10, MASK_8B, 0, 0, 7)]
    pub fn adc_xl_cb(&mut self, arg1: u8, _arg2: u8) {
        let tmp = self.xl() as u16 + arg1 as u16 + (self.flags.get(Flag::C) as u8) as u16;
        self.x = ((tmp & 0xf) as u16) | (self.xh() << 4) | (self.xp() << 8);
        self.flags.set(Flag::C, (tmp >> 4) != 0);
        self.flags.set(Flag::Z, (tmp & 0xf) == 0);
    }
    /// ADC YH: Add arg1 and carry to YH.
    // #[instruction("ADC  YH #0x{:02X}         ", 0xA20, MASK_8B, 0, 0, 7)]
    pub fn adc_yh_cb(&mut self, arg1: u8, _arg2: u8) {
        let tmp = self.yh() as u16 + arg1 as u16 + (self.flags.get(Flag::C) as u8) as u16;
        self.y = self.yl() | (((tmp & 0xf) as u16) << 4) | (self.yp() << 8);
        self.flags.set(Flag::C, (tmp >> 4) != 0);
        self.flags.set(Flag::Z, (tmp & 0xf) == 0);
    }
    /// ADC YL: Add arg1 and carry to YL.
    // #[instruction("ADC  YL #0x{:02X}         ", 0xA30, MASK_8B, 0, 0, 7)]
    pub fn adc_yl_cb(&mut self, arg1: u8, _arg2: u8) {
        let tmp = self.yl() as u16 + arg1 as u16 + (self.flags.get(Flag::C) as u8) as u16;
        self.y = ((tmp & 0xf) as u16) | (self.yh() << 4) | (self.yp() << 8);
        self.flags.set(Flag::C, (tmp >> 4) != 0);
        self.flags.set(Flag::Z, (tmp & 0xf) == 0);
    }
    /// CP XH: Compare XH with arg1.
    // #[instruction("CP   XH #0x{:02X}         ", 0xA40, MASK_8B, 0, 0, 7)]
    pub fn cp_xh_cb(&mut self, arg1: u8, _arg2: u8) {
        self.flags.set(Flag::C, (self.xh() as u16) < (arg1 as u16));
        self.flags.set(Flag::Z, (self.xh() as u16) == (arg1 as u16));
    }
    /// CP XL: Compare XL with arg1.
    // #[instruction("CP   XL #0x{:02X}         ", 0xA50, MASK_8B, 0, 0, 7)]
    pub fn cp_xl_cb(&mut self, arg1: u8, _arg2: u8) {
        self.flags.set(Flag::C, (self.xl() as u16) < (arg1 as u16));
        self.flags.set(Flag::Z, (self.xl() as u16) == (arg1 as u16));
    }
    /// CP YH: Compare YH with arg1.
    // #[instruction("CP   YH #0x{:02X}         ", 0xA60, MASK_8B, 0, 0, 7)]
    pub fn cp_yh_cb(&mut self, arg1: u8, _arg2: u8) {
        self.flags.set(Flag::C, (self.yh() as u16) < (arg1 as u16));
        self.flags.set(Flag::Z, (self.yh() as u16) == (arg1 as u16));
    }
    /// CP YL: Compare YL with arg1.
    // #[instruction("CP   YL #0x{:02X}         ", 0xA70, MASK_8B, 0, 0, 7)]
    pub fn cp_yl_cb(&mut self, arg1: u8, _arg2: u8) {
        self.flags.set(Flag::C, (self.yl() as u16) < (arg1 as u16));
        self.flags.set(Flag::Z, (self.yl() as u16) == (arg1 as u16));
    }
    /// LD R({:X}) #imm: Load immediate value into register or memory specified by arg1.
    // #[instruction("LD   R({:X}) #0x{:02X}       ", 0xE00, MASK_6B, 4, 0x030, 5)]
    pub fn ld_r_i_cb(&mut self, arg1: u8, arg2: u8) {
        self.set_rq(arg1 as u16, arg2);
    }
    /// LD R({:X}) Q({:X}): Load value from register/memory Q(arg2) into R(arg1).
    // #[instruction("LD   R({:X}) Q({:X})        ", 0xEC0, MASK_8B, 2, 0x00C, 5)]
    pub fn ld_r_q_cb(&mut self, arg1: u8, arg2: u8) {
        let rq = self.get_rq(arg2 as u16);
        self.set_rq(arg1 as u16, rq);
    }
    /// LD A, M(#imm): Load memory at address arg1 into A.
    // #[instruction("LD   A M(#0x{:02X})       ", 0xFA0, MASK_8B, 0, 0, 5)]
    pub fn ld_a_mn_cb(&mut self, arg1: u8, _arg2: u8) {
        self.a = self.get_mem(arg1 as usize);
    }
    /// LD B, M(#imm): Load memory at address arg1 into B.
    // #[instruction("LD   B M(#0x{:02X})       ", 0xFB0, MASK_8B, 0, 0, 5)]
    pub fn ld_b_mn_cb(&mut self, arg1: u8, _arg2: u8) {
        self.b = self.get_mem(arg1 as usize);
    }
    /// LD M(#imm), A: Store A into memory at address arg1.
    // #[instruction("LD   M(#0x{:02X}) A       ", 0xF80, MASK_8B, 0, 0, 5)]
    pub fn ld_mn_a_cb(&mut self, arg1: u8, _arg2: u8) {
        self.set_mem(arg1 as usize, self.a);
    }
    /// LD M(#imm), B: Store B into memory at address arg1.
    // #[instruction("LD   M(#0x{:02X}) B       ", 0xF90, MASK_8B, 0, 0, 5)]
    pub fn ld_mn_b_cb(&mut self, arg1: u8, _arg2: u8) {
        self.set_mem(arg1 as usize, self.b);
    }
    /// LDPX MX: Store arg1 at memory[X], increment X.
    // #[instruction("LDPX MX #0x{:02X}         ", 0xE60, MASK_8B, 0, 0, 5)]
    pub fn ldpx_mx_cb(&mut self, arg1: u8, _arg2: u8) {
        self.set_mem(self.x as usize, arg1);
        self.x = ((self.x + 1) & 0xff) | (self.xp() << 8);
    }
    /// LDPX R Q: Store value from Q(arg2) into R(arg1), increment X.
    // #[instruction("LDPX R({:X}) Q({:X})        ", 0xEE0, MASK_8B, 2, 0x00C, 5)]
    pub fn ldpx_r_cb(&mut self, arg1: u8, arg2: u8) {
        let rq = self.get_rq(arg2 as u16);
        self.set_rq(arg1 as u16, rq);
        self.x = ((self.x + 1) & 0xff) | (self.xp() << 8);
    }
    /// LDPY MY: Store arg1 at memory[Y], increment Y.
    // #[instruction("LDPY MY #0x{:02X}         ", 0xE70, MASK_8B, 0, 0, 5)]
    pub fn ldpy_my_cb(&mut self, arg1: u8, _arg2: u8) {
        self.set_mem(self.y as usize, arg1);
        self.y = ((self.y + 1) & 0xff) | (self.yp() << 8);
    }
    /// LDPY R Q: Store value from Q(arg2) into R(arg1), increment Y.
    // #[instruction("LDPY R({:X}) Q({:X})        ", 0xEF0, MASK_8B, 2, 0x00C, 5)]
    pub fn ldpy_r_cb(&mut self, arg1: u8, arg2: u8) {
        let rq = self.get_rq(arg2 as u16);
        self.set_rq(arg1 as u16, rq);
        self.y = ((self.y + 1) & 0xff) | (self.yp() << 8);
    }
    /// LBPX: Store lower and upper nibbles of arg1 at memory[X] and memory[X+1], increment X by 2.
    // #[instruction("LBPX #0x{:02X}            ", 0x900, MASK_4B, 0, 0, 5)]
    pub fn lbpx_cb(&mut self, arg1: u8, _arg2: u8) {
        self.set_mem(self.x as usize, arg1 & 0xf);
        self.set_mem(
            (((self.x + 1) & 0xff) as usize) | ((self.xp() as usize) << 8),
            (arg1 >> 4) & 0xf,
        );
        self.x = ((self.x + 2) & 0xff) | (self.xp() << 8);
    }
    /// SET: Set flag bits specified by arg1.
    // #[instruction("SET  #0x{:02X}            ", 0xF40, MASK_8B, 0, 0, 7)]
    pub fn set_cb(&mut self, arg1: u8, _arg2: u8) {
        self.flags.0 |= arg1;
    }
    /// RST: Reset flag bits specified by arg1.
    // #[instruction("RST  #0x{:02X}            ", 0xF50, MASK_8B, 0, 0, 7)]
    pub fn rst_cb(&mut self, arg1: u8, _arg2: u8) {
        self.flags.0 &= arg1;
    }
    /// SCF: Set carry flag.
    // #[instruction("SCF                   ", 0xF41, MASK_12B, 0, 0, 7)]
    pub fn scf_cb(&mut self, _arg1: u8, _arg2: u8) {
        self.flags.set(Flag::C, true);
    }
    /// RCF: Reset carry flag.
    // #[instruction("RCF                   ", 0xF5E, MASK_12B, 0, 0, 7)]
    pub fn rcf_cb(&mut self, _arg1: u8, _arg2: u8) {
        self.flags.set(Flag::C, false);
    }
    /// SZF: Set zero flag.
    // #[instruction("SZF                   ", 0xF42, MASK_12B, 0, 0, 7)]
    pub fn szf_cb(&mut self, _arg1: u8, _arg2: u8) {
        self.flags.set(Flag::Z, true);
    }
    /// RZF: Reset zero flag.
    // #[instruction("RZF                   ", 0xF5D, MASK_12B, 0, 0, 7)]
    pub fn rzf_cb(&mut self, _arg1: u8, _arg2: u8) {
        self.flags.set(Flag::Z, false);
    }
    /// SDF: Set decimal flag.
    // #[instruction("SDF                   ", 0xF44, MASK_12B, 0, 0, 7)]
    pub fn sdf_cb(&mut self, _arg1: u8, _arg2: u8) {
        self.flags.set(Flag::D, true);
    }
    /// RDF: Reset decimal flag.
    // #[instruction("RDF                   ", 0xF5B, MASK_12B, 0, 0, 7)]
    pub fn rdf_cb(&mut self, _arg1: u8, _arg2: u8) {
        self.flags.set(Flag::D, false);
    }
    /// EI: Enable interrupts (set I flag).
    // #[instruction("EI                    ", 0xF48, MASK_12B, 0, 0, 7)]
    pub fn ei_cb(&mut self, _arg1: u8, _arg2: u8) {
        self.flags.set(Flag::I, true);
    }
    /// DI: Disable interrupts (reset I flag).
    // #[instruction("DI                    ", 0xF57, MASK_12B, 0, 0, 7)]
    pub fn di_cb(&mut self, _arg1: u8, _arg2: u8) {
        self.flags.set(Flag::I, false);
    }
    /// INC SP: Increment stack pointer.
    // #[instruction("INC  SP               ", 0xFDB, MASK_12B, 0, 0, 5)]
    pub fn inc_sp_cb(&mut self, _arg1: u8, _arg2: u8) {
        self.sp = self.sp.wrapping_add(1);
    }
    /// DEC SP: Decrement stack pointer.
    // #[instruction("DEC  SP               ", 0xFCB, MASK_12B, 0, 0, 5)]
    pub fn dec_sp_cb(&mut self, _arg1: u8, _arg2: u8) {
        self.sp = self.sp.wrapping_sub(1);
    }
    /// PUSH R: Push register/memory specified by arg1 onto stack.
    // #[instruction("PUSH R({:X})             ", 0xFC0, MASK_10B, 0, 0, 5)]
    pub fn push_r_cb(&mut self, arg1: u8, _arg2: u8) {
        self.sp = self.sp.wrapping_sub(1);
        let rq = self.get_rq(arg1 as u16);
        self.set_mem(self.sp as usize, rq);
    }
    /// PUSH XP: Push XP onto stack.
    // #[instruction("PUSH XP               ", 0xFC4, MASK_12B, 0, 0, 5)]
    pub fn push_xp_cb(&mut self, _arg1: u8, _arg2: u8) {
        self.sp = self.sp.wrapping_sub(1);
        self.set_mem(self.sp as usize, self.xp() as u8);
    }
    /// PUSH XH: Push XH onto stack.
    // #[instruction("PUSH XH               ", 0xFC5, MASK_12B, 0, 0, 5)]
    pub fn push_xh_cb(&mut self, _arg1: u8, _arg2: u8) {
        self.sp = self.sp.wrapping_sub(1);
        self.set_mem(self.sp as usize, self.xh() as u8);
    }
    /// PUSH XL: Push XL onto stack.
    // #[instruction("PUSH XL               ", 0xFC6, MASK_12B, 0, 0, 5)]
    pub fn push_xl_cb(&mut self, _arg1: u8, _arg2: u8) {
        self.sp = self.sp.wrapping_sub(1);
        self.set_mem(self.sp as usize, self.xl() as u8);
    }
    /// PUSH YP: Push YP onto stack.
    // #[instruction("PUSH YP               ", 0xFC7, MASK_12B, 0, 0, 5)]
    pub fn push_yp_cb(&mut self, _arg1: u8, _arg2: u8) {
        self.sp = self.sp.wrapping_sub(1);
        self.set_mem(self.sp as usize, self.yp() as u8);
    }
    /// PUSH YH: Push YH onto stack.
    // #[instruction("PUSH YH               ", 0xFC8, MASK_12B, 0, 0, 5)]
    pub fn push_yh_cb(&mut self, _arg1: u8, _arg2: u8) {
        self.sp = self.sp.wrapping_sub(1);
        self.set_mem(self.sp as usize, self.yh() as u8);
    }
    /// PUSH YL: Push YL onto stack.
    // #[instruction("PUSH YL               ", 0xFC9, MASK_12B, 0, 0, 5)]
    pub fn push_yl_cb(&mut self, _arg1: u8, _arg2: u8) {
        self.sp = self.sp.wrapping_sub(1);
        self.set_mem(self.sp as usize, self.yl() as u8);
    }
    /// PUSH F: Push flags onto stack.
    // #[instruction("PUSH F                ", 0xFCA, MASK_12B, 0, 0, 5)]
    pub fn push_f_cb(&mut self, _arg1: u8, _arg2: u8) {
        self.sp = self.sp.wrapping_sub(1);
        self.set_mem(self.sp as usize, self.flags.0 as u8);
    }
    /// POP R: Pop value from stack into register/memory specified by arg1.
    // #[instruction("POP  R({:X})             ", 0xFD0, MASK_10B, 0, 0, 5)]
    pub fn pop_r_cb(&mut self, arg1: u8, _arg2: u8) {
        let value = self.get_mem(self.sp as usize);
        self.set_rq(arg1 as u16, value);
        self.sp = self.sp.wrapping_add(1);
    }
    /// POP XP: Pop value from stack into XP.
    // #[instruction("POP  XP               ", 0xFD4, MASK_12B, 0, 0, 5)]
    pub fn pop_xp_cb(&mut self, _arg1: u8, _arg2: u8) {
        let value = self.get_mem(self.sp as usize) as u16;
        self.x = self.xl() | (self.xh() << 4) | (value << 8);
        self.sp = self.sp.wrapping_add(1);
    }
    /// POP XH: Pop value from stack into XH.
    // #[instruction("POP  XH               ", 0xFD5, MASK_12B, 0, 0, 5)]
    pub fn pop_xh_cb(&mut self, _arg1: u8, _arg2: u8) {
        let value = self.get_mem(self.sp as usize) as u16;
        self.x = self.xl() | (value << 4) | (self.xp() << 8);
        self.sp = self.sp.wrapping_add(1);
    }
    /// POP XL: Pop value from stack into XL.
    // #[instruction("POP  XL               ", 0xFD6, MASK_12B, 0, 0, 5)]
    pub fn pop_xl_cb(&mut self, _arg1: u8, _arg2: u8) {
        let value = self.get_mem(self.sp as usize) as u16;
        self.x = value | (self.xh() << 4) | (self.xp() << 8);
        self.sp = self.sp.wrapping_add(1);
    }
    /// POP YP: Pop value from stack into YP.
    // #[instruction("POP  YP               ", 0xFD7, MASK_12B, 0, 0, 5)]
    pub fn pop_yp_cb(&mut self, _arg1: u8, _arg2: u8) {
        let value = self.get_mem(self.sp as usize) as u16;
        //self.y = value | (self.yh() << 4) | (self.yp() << 8);
        self.y = self.yl() | (self.yh() << 4) | (value << 8);
        self.sp = self.sp.wrapping_add(1);
    }
    /// POP YH: Pop value from stack into YH.
    // #[instruction("POP  YH               ", 0xFD8, MASK_12B, 0, 0, 5)]
    pub fn pop_yh_cb(&mut self, _arg1: u8, _arg2: u8) {
        let value = self.get_mem(self.sp as usize) as u16;
        self.y = self.yl() | (value << 4) | (self.yp() << 8);
        self.sp = self.sp.wrapping_add(1);
    }
    /// POP YL: Pop value from stack into YL.
    // #[instruction("POP  YL               ", 0xFD9, MASK_12B, 0, 0, 5)]
    pub fn pop_yl_cb(&mut self, _arg1: u8, _arg2: u8) {
        let value = self.get_mem(self.sp as usize) as u16;
        self.y = value | (self.yh() << 4) | (self.yp() << 8);
        self.sp = self.sp.wrapping_add(1);
    }
    /// POP F: Pop value from stack into flags.
    // #[instruction("POP  F                ", 0xFDA, MASK_12B, 0, 0, 5)]
    pub fn pop_f_cb(&mut self, _arg1: u8, _arg2: u8) {
        self.flags.0 = self.get_mem(self.sp as usize);
        self.sp = self.sp.wrapping_add(1);
    }
    /// LD SPH R: Load SPH from register specified by arg1.
    // #[instruction("LD   SPH R({:X})         ", 0xFE0, MASK_10B, 0, 0, 5)]
    pub fn ld_sph_r_cb(&mut self, arg1: u8, _arg2: u8) {
        self.sp = self.spl() | (self.get_rq(arg1 as u16) << 4);
    }
    /// LD SPL R: Load SPL from register specified by arg1.
    // #[instruction("LD   SPL R({:X})         ", 0xFF0, MASK_10B, 0, 0, 5)]
    pub fn ld_spl_r_cb(&mut self, arg1: u8, _arg2: u8) {
        self.sp = self.get_rq(arg1 as u16) | (self.sph() << 4);
    }
    /// LD R SPH: Store SPH into register specified by arg1.
    // #[instruction("LD   R({:X}) SPH     ", 0xFE4, MASK_10B, 0, 0, 5)]
    pub fn ld_r_sph_cb(&mut self, arg1: u8, _arg2: u8) {
        self.set_rq(arg1 as u16, self.sph());
    }
    /// LD R SPL: Store SPL into register specified by arg1.
    // #[instruction("LD   R({:X}) SPL     ", 0xFF4, MASK_10B, 0, 0, 5)]
    pub fn ld_r_spl_cb(&mut self, arg1: u8, _arg2: u8) {
        self.set_rq(arg1 as u16, self.spl());
    }
    /// ADD R #imm: Add immediate value to register/memory specified by arg1.
    // #[instruction("ADD  R({:X}) #0x{:02X}       ", 0xC00, MASK_6B, 4, 0x030, 7)]
    pub fn add_r_i_cb(&mut self, arg1: u8, arg2: u8) {
        let tmp = self.get_rq(arg1 as u16) as u16 + arg2 as u16;
        if self.flags.get(Flag::D) {
            if tmp >= 10 {
                self.set_rq(arg1 as u16, ((tmp - 10) as u8) & 0xf);
                self.flags.set(Flag::C, true);
            } else {
                self.set_rq(arg1 as u16, tmp as u8);
                self.flags.set(Flag::C, false);
            }
        } else {
            self.set_rq(arg1 as u16, (tmp as u8) & 0xf);
            self.flags.set(Flag::C, (tmp >> 4) != 0);
        }
        let rq = self.get_rq(arg1 as u16);
        self.flags.set(Flag::Z, rq == 0);
    }
    /// ADD R Q: Add value from Q(arg2) to R(arg1).
    // #[instruction("ADD  R({:X}) Q({:X})        ", 0xA80, MASK_8B, 2, 0x00C, 7)]
    pub fn add_r_q_cb(&mut self, arg1: u8, arg2: u8) {
        let tmp = (self.get_rq(arg1 as u16) as u16) + (self.get_rq(arg2 as u16) as u16);
        if self.flags.get(Flag::D) {
            if tmp >= 10 {
                self.set_rq(arg1 as u16, ((tmp - 10) as u8) & 0xf);
                self.flags.set(Flag::C, true);
            } else {
                self.set_rq(arg1 as u16, tmp as u8);
                self.flags.set(Flag::C, false);
            }
        } else {
            self.set_rq(arg1 as u16, (tmp as u8) & 0xf);
            self.flags.set(Flag::C, (tmp >> 4) != 0);
        }
        let rq = self.get_rq(arg1 as u16);
        self.flags.set(Flag::Z, rq == 0);
    }
    /// ADC R #imm: Add immediate value and carry to register/memory specified by arg1.
    // #[instruction("ADC  R({:X}) #0x{:02X}       ", 0xC40, MASK_6B, 4, 0x030, 7)]
    pub fn adc_r_i_cb(&mut self, arg1: u8, arg2: u8) {
        let tmp =
            (self.get_rq(arg1 as u16) as u16) + arg2 as u16 + (self.flags.get(Flag::C) as u16);
        if self.flags.get(Flag::D) {
            if tmp >= 10 {
                self.set_rq(arg1 as u16, ((tmp - 10) as u8) & 0xf);
                self.flags.set(Flag::C, true);
            } else {
                self.set_rq(arg1 as u16, tmp as u8);
                self.flags.set(Flag::C, false);
            }
        } else {
            self.set_rq(arg1 as u16, (tmp as u8) & 0xf);
            self.flags.set(Flag::C, (tmp >> 4) != 0);
        }
        let rq = self.get_rq(arg1 as u16);
        self.flags.set(Flag::Z, rq == 0);
    }
    /// ADC R Q: Add value from Q(arg2) and carry to R(arg1).
    // #[instruction("ADC  R({:X}) Q({:X})        ", 0xA90, MASK_8B, 2, 0x00C, 7)]
    pub fn adc_r_q_cb(&mut self, arg1: u8, arg2: u8) {
        let tmp = (self.get_rq(arg1 as u16) as u16)
            + (self.get_rq(arg2 as u16) as u16)
            + (self.flags.get(Flag::C) as u16);
        if self.flags.get(Flag::D) {
            if tmp >= 10 {
                self.set_rq(arg1 as u16, ((tmp - 10) as u8) & 0xf);
                self.flags.set(Flag::C, true);
            } else {
                self.set_rq(arg1 as u16, tmp as u8);
                self.flags.set(Flag::C, false);
            }
        } else {
            self.set_rq(arg1 as u16, (tmp as u8) & 0xf);
            self.flags.set(Flag::C, (tmp >> 4) != 0);
        }
        let rq = self.get_rq(arg1 as u16);
        self.flags.set(Flag::Z, rq == 0);
    }
    /// SUB R Q: Subtract arg2 from register/memory specified by arg1.
    // #[instruction("SUB  R({:X}) Q({:X})        ", 0xAA0, MASK_8B, 2, 0x00C, 7)]
    pub fn sub_cb(&mut self, arg1: u8, arg2: u8) {
        let tmp = (self.get_rq(arg1 as u16) as u16) - (self.get_rq(arg2 as u16) as u16);
        if self.flags.get(Flag::D) {
            if (tmp >> 4) != 0 {
                self.set_rq(arg1 as u16, (tmp.wrapping_sub(6) as u8) & 0xf);
            } else {
                self.set_rq(arg1 as u16, tmp as u8);
            }
        } else {
            self.set_rq(arg1 as u16, (tmp as u8) & 0xf);
        }
        self.flags.set(Flag::C, (tmp >> 4) != 0);
        let rq = self.get_rq(arg1 as u16);
        self.flags.set(Flag::Z, rq == 0);
    }
    /// SBC R #imm: Subtract immediate value and carry from register/memory specified by arg1.
    // #[instruction("SBC  R({:X}) #0x{:02X}       ", 0xD40, MASK_6B, 4, 0x030, 7)]
    pub fn sbc_r_i_cb(&mut self, arg1: u8, arg2: u8) {
        let tmp =
            (self.get_rq(arg1 as u16) as u16) - arg2 as u16 - (self.flags.get(Flag::C) as u16);
        if self.flags.get(Flag::D) {
            if (tmp >> 4) != 0 {
                self.set_rq(arg1 as u16, ((tmp.wrapping_sub(6)) as u8) & 0xf);
            } else {
                self.set_rq(arg1 as u16, tmp as u8);
            }
        } else {
            self.set_rq(arg1 as u16, (tmp as u8) & 0xf);
        }
        self.flags.set(Flag::C, (tmp >> 4) != 0);
        let rq = self.get_rq(arg1 as u16);
        self.flags.set(Flag::Z, rq == 0);
    }
    /// SBC R Q: Subtract value from Q(arg2) and carry from R(arg1).
    // #[instruction("SBC  R({:X}) Q({:X})        ", 0xAB0, MASK_8B, 2, 0x00C, 7)]
    pub fn sbc_r_q_cb(&mut self, arg1: u8, arg2: u8) {
        let tmp = (self.get_rq(arg1 as u16) as u16)
            - (self.get_rq(arg2 as u16) as u16)
            - (self.flags.get(Flag::C) as u16);
        if self.flags.get(Flag::D) {
            if (tmp >> 4) != 0 {
                self.set_rq(arg1 as u16, ((tmp.wrapping_sub(6)) as u8) & 0xf);
            } else {
                self.set_rq(arg1 as u16, tmp as u8);
            }
        } else {
            self.set_rq(arg1 as u16, (tmp as u8) & 0xf);
        }
        self.flags.set(Flag::C, (tmp >> 4) != 0);
        let rq = self.get_rq(arg1 as u16);
        self.flags.set(Flag::Z, rq == 0);
    }
    /// AND R #imm: Bitwise AND immediate value with register/memory specified by arg1.
    // #[instruction("AND  R({:X}) #0x{:02X}       ", 0xC80, MASK_6B, 4, 0x030, 7)]
    pub fn and_r_i_cb(&mut self, arg1: u8, arg2: u8) {
        let mut rq = self.get_rq(arg1 as u16);
        self.set_rq(arg1 as u16, rq & arg2);
        rq = self.get_rq(arg1 as u16);
        
        self.flags.set(Flag::Z, rq == 0);
    }
    /// AND R Q: Bitwise AND value from Q(arg2) with R(arg1).
    // #[instruction("AND  R({:X}) Q({:X})        ", 0xAC0, MASK_8B, 2, 0x00C, 7)]
    pub fn and_r_q_cb(&mut self, arg1: u8, arg2: u8) {
        let rq1 = self.get_rq(arg1 as u16);
        let rq2 = self.get_rq(arg2 as u16);
        self.set_rq(arg1 as u16, rq1 & rq2);
        let rq = self.get_rq(arg1 as u16);
        self.flags.set(Flag::Z, rq == 0);
    }
    /// OR R #imm: Bitwise OR immediate value with register/memory specified by arg1.
    // #[instruction("OR   R({:X}) #0x{:02X}       ", 0xCC0, MASK_6B, 4, 0x030, 7)]
    pub fn or_r_i_cb(&mut self, arg1: u8, arg2: u8) {
        let mut rq = self.get_rq(arg1 as u16);
        self.set_rq(arg1 as u16, rq | arg2);
        rq = self.get_rq(arg1 as u16);
        self.flags.set(Flag::Z, rq == 0);
    }
    /// OR R Q: Bitwise OR value from Q(arg2) with R(arg1).
    // #[instruction("OR   R({:X}) Q({:X})        ", 0xAD0, MASK_8B, 2, 0x00C, 7)]
    pub fn or_r_q_cb(&mut self, arg1: u8, arg2: u8) {
        let rq1 = self.get_rq(arg1 as u16);
        let rq2 = self.get_rq(arg2 as u16);
        self.set_rq(arg1 as u16, rq1 | rq2);
        let rq = self.get_rq(arg1 as u16);
        self.flags.set(Flag::Z, rq == 0);
    }
    /// XOR R #imm: Bitwise XOR immediate value with register/memory specified by arg1.
    // #[instruction("XOR  R({:X}) #0x{:02X}       ", 0xD00, MASK_6B, 4, 0x030, 7)]
    pub fn xor_r_i_cb(&mut self, arg1: u8, arg2: u8) {
        let rq1 = self.get_rq(arg1 as u16);
        self.set_rq(arg1 as u16, rq1 ^ arg2);
        let rq = self.get_rq(arg1 as u16);
        self.flags.set(Flag::Z, rq == 0);
    }
    /// XOR R Q: Bitwise XOR value from Q(arg2) with R(arg1).
    // #[instruction("XOR  R({:X}) Q({:X})        ", 0xAE0, MASK_8B, 2, 0x00C, 7)]
    pub fn xor_r_q_cb(&mut self, arg1: u8, arg2: u8) {
        let rq1 = self.get_rq(arg1 as u16);
        let rq2 = self.get_rq(arg2 as u16);
        self.set_rq(
            arg1 as u16,
            rq1 ^ rq2
        );
        let rq = self.get_rq(arg1 as u16);
        self.flags.set(Flag::Z, rq == 0);
    }
    /// CP R #imm: Compare register/memory specified by arg1 with immediate value.
    // #[instruction("CP   R({:X}) #0x{:02X}       ", 0xDC0, MASK_6B, 4, 0x030, 7)]
    pub fn cp_r_i_cb(&mut self, arg1: u8, arg2: u8) {
        let mut rq = self.get_rq(arg1 as u16);
        self.flags.set(Flag::C, rq < arg2);
        rq = self.get_rq(arg1 as u16);
        self.flags.set(Flag::Z, rq == arg2);
    }
    /// CP R Q: Compare register/memory specified by arg1 with value from Q(arg2).
    // #[instruction("CP   R({:X}) Q({:X})        ", 0xF00, MASK_8B, 2, 0x00C, 7)]
    pub fn cp_r_q_cb(&mut self, arg1: u8, arg2: u8) {
        let mut rq1 = self.get_rq(arg1 as u16);
        let mut rq2 = self.get_rq(arg2 as u16);
        self.flags
            .set(Flag::C, rq1 < rq2);
        rq1 = self.get_rq(arg1 as u16);
        rq2 = self.get_rq(arg2 as u16);
        self.flags.set(
            Flag::Z,
            rq1 == rq2,
        );
    }
    /// FAN R #imm: Set Z flag if (R & #imm) == 0.
    // #[instruction("FAN  R({:X}) #0x{:02X}       ", 0xD80, MASK_6B, 4, 0x030, 7)]
    pub fn fan_r_i_cb(&mut self, arg1: u8, arg2: u8) {
        let rq = self.get_rq(arg1 as u16);
        self.flags
            .set(Flag::Z, (rq & arg2) == 0);
    }
    /// FAN R Q: Set Z flag if (R & Q) == 0.
    // #[instruction("FAN  R({:X}) Q({:X})        ", 0xF10, MASK_8B, 2, 0x00C, 7)]
    pub fn fan_r_q_cb(&mut self, arg1: u8, arg2: u8) {
        let rq1 = self.get_rq(arg1 as u16);
        let rq2 = self.get_rq(arg2 as u16);
        self.flags.set(
            Flag::Z,
            (rq1 & rq2) == 0,
        );
    }
    /// RLC R: Rotate left through carry.
    // #[instruction("RLC  R({:X})             ", 0xAF0, MASK_8B, 0, 0, 7)]
    pub fn rlc_cb(&mut self, arg1: u8, _arg2: u8) {
        let rq = self.get_rq(arg1 as u16);
        let tmp = (rq << 1) | (self.flags.get(Flag::C) as u8);
        self.flags
            .set(Flag::C, (rq & 0x8) != 0);
        self.set_rq(arg1 as u16, tmp & 0xf)
    }
    /// RRC R: Rotate right through carry.
    // #[instruction("RRC  R({:X})             ", 0xE8C, MASK_10B, 0, 0, 5)]
    pub fn rrc_cb(&mut self, arg1: u8, _arg2: u8) {
        let rq = self.get_rq(arg1 as u16);
        let tmp = (rq >> 1) | (self.flags.get(Flag::C) as u8) << 3;
        self.flags
            .set(Flag::C, (rq & 0x1) != 0);
        self.set_rq(arg1 as u16, tmp & 0xf);
    }
    /// INC M(#imm): Increment memory at address arg1.
    // #[instruction("INC  M(#0x{:02X})         ", 0xF60, MASK_8B, 0, 0, 7)]
    pub fn inc_mn_cb(&mut self, arg1: u8, _arg2: u8) {
        let tmp = self.get_mem(arg1 as usize) as u16 + 1;
        self.set_mem(arg1 as usize, (tmp & 0xf) as u8);
        self.flags.set(Flag::C, (tmp >> 4) != 0);
        let mem_at_arg1 = self.get_mem(arg1 as usize);
        self.flags.set(Flag::Z, mem_at_arg1 == 0);
    }
    /// DEC M(#imm): Decrement memory at address arg1.
    // #[instruction("DEC  M(#0x{:02X})         ", 0xF70, MASK_8B, 0, 0, 7)]
    pub fn dec_mn_cb(&mut self, arg1: u8, _arg2: u8) {
        let tmp = self.get_mem(arg1 as usize) as u16 - 1;
        self.set_mem(arg1 as usize, (tmp & 0xf) as u8);
        let mem_at_arg1 = self.get_mem(arg1 as usize);
        self.flags.set(Flag::C, (tmp >> 4) != 0);
        self.flags.set(Flag::Z, mem_at_arg1 == 0);
    }
    /// ACPX R: Add memory[X], R(arg1), and carry, store in memory[X], increment X.
    // #[instruction("ACPX R({:X})             ", 0xF28, MASK_10B, 0, 0, 7)]
    pub fn acpx_cb(&mut self, arg1: u8, _arg2: u8) {
        let tmp = self.get_mem(self.x as usize)
            + self.get_rq(arg1 as u16)
            + (self.flags.get(Flag::C) as u8);
        if self.flags.get(Flag::D) {
            if tmp >= 10 {
                self.set_mem(self.x as usize, tmp.wrapping_sub(10) & 0xf);
                self.flags.set(Flag::C, true);
            } else {
                self.set_mem(self.x as usize, tmp);
                self.flags.set(Flag::C, false);
            }
        } else {
            self.set_mem(self.x as usize, (tmp & 0xf) as u8);
            self.flags.set(Flag::C, (tmp >> 4) != 0);
        }
        let mem_at_x = self.get_mem(self.x as usize);
        self.flags.set(Flag::Z, mem_at_x == 0);
        self.x = ((self.x + 1) & 0xff) | (self.xp() << 8);
    }
    /// ACPY R: Add memory[Y], R(arg1), and carry, store in memory[Y], increment Y.
    // #[instruction("ACPY R({:X})             ", 0xF2C, MASK_10B, 0, 0, 7)]
    pub fn acpy_cb(&mut self, arg1: u8, _arg2: u8) {
        let tmp = self.get_mem(self.y as usize)
            + self.get_rq(arg1 as u16)
            + (self.flags.get(Flag::C) as u8);
        if self.flags.get(Flag::D) {
            if tmp >= 10 {
                self.set_mem(self.y as usize, tmp.wrapping_sub(10) & 0xf);
                self.flags.set(Flag::C, true);
            } else {
                self.set_mem(self.y as usize, tmp as u8);
                self.flags.set(Flag::C, false);
            }
        } else {
            self.set_mem(self.y as usize, (tmp & 0xf) as u8);
            self.flags.set(Flag::C, (tmp >> 4) != 0);
        }
        let mem_at_y = self.get_mem(self.y as usize);
        self.flags.set(Flag::Z, mem_at_y == 0);
        self.y = ((self.y + 1) & 0xff) | (self.yp() << 8);
    }
    /// SCPX R: Subtract R(arg1) and carry from memory[X], store in memory[X], increment X.
    // #[instruction("SCPX R({:X})             ", 0xF38, MASK_10B, 0, 0, 7)]
    pub fn scpx_cb(&mut self, arg1: u8, _arg2: u8) {
        let tmp = self.get_mem(self.x as usize)
            - self.get_rq(arg1 as u16)
            - (self.flags.get(Flag::C) as u8);
        if self.flags.get(Flag::D) {
            if (tmp >> 4) != 0 {
                self.set_mem(self.x as usize, tmp.wrapping_sub(6) & 0xf);
            } else {
                self.set_mem(self.x as usize, tmp);
            }
        } else {
            self.set_mem(self.x as usize, (tmp & 0xf) as u8);
        }
        self.flags.set(Flag::C, (tmp >> 4) != 0);
        let mem_at_x = self.get_mem(self.x as usize);
        self.flags.set(Flag::Z, mem_at_x == 0);
        self.x = ((self.x + 1) & 0xff) | (self.xp() << 8);
    }
    /// SCPY R: Subtract R(arg1) and carry from memory[Y], store in memory[Y], increment Y.
    // #[instruction("SCPY R({:X})             ", 0xF3C, MASK_10B, 0, 0, 7)]
    pub fn scpy_cb(&mut self, arg1: u8, _arg2: u8) {
        let tmp = self.get_mem(self.y as usize)
            - self.get_rq(arg1 as u16)
            - (self.flags.get(Flag::C) as u8);

        if self.flags.get(Flag::D) {
            if (tmp >> 4) != 0 {
                self.set_mem(self.y as usize, tmp.wrapping_sub(6) & 0xf);
            } else {
                self.set_mem(self.y as usize, tmp);
            }
        } else {
            self.set_mem(self.y as usize, (tmp & 0xf) as u8);
        }
        self.flags.set(Flag::C, (tmp >> 4) != 0);
        let mem_at_y = self.get_mem(self.y as usize);
        self.flags.set(Flag::Z, mem_at_y == 0);
        self.y = ((self.y + 1) & 0xff) | (self.yp() << 8);
    }
    /// NOT R: Bitwise NOT on register/memory specified by arg1.
    // #[instruction("NOT  R({:X})             ", 0xD0F, 0xFCF, 4, 0, 7)]
    pub fn not_cb(&mut self, arg1: u8, _arg2: u8) {
        let mut rq = self.get_rq(arg1 as u16);
        self.set_rq(arg1 as u16, !rq & 0xf);
        rq = self.get_rq(arg1 as u16);
        self.flags.set(Flag::Z, rq == 0);
    }
}
