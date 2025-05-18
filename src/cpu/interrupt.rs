use std::fmt;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum InterruptId {
    ProgTimer = 0,
    Serial = 1,
    K10K13 = 2,
    K00K03 = 3,
    Stopwatch = 4,
    ClockTimer = 5,
}

impl fmt::Display for InterruptId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s = match self {
            InterruptId::ProgTimer => "ProgTimer",
            InterruptId::Serial => "Serial",
            InterruptId::K10K13 => "K10K13",
            InterruptId::K00K03 => "K00K03",
            InterruptId::Stopwatch => "Stopwatch",
            InterruptId::ClockTimer => "ClockTimer",
        };
        write!(f, "{}", s)
    }
}

#[derive(Debug)]
pub struct Interrupt {
    pub id: InterruptId,
    pub triggered: bool,
    pub factor: u8,
    pub mask: u8,
    pub vector: u8,
}

impl Interrupt {
    pub fn new(id: InterruptId, vector: u8) -> Self {
        Self {
            id,
            triggered: false,
            factor: 0,
            mask: 0,
            vector,
        }
    }
}

impl fmt::Display for Interrupt {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.id)
    }
}