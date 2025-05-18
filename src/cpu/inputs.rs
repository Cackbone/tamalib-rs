#[repr(u8)]
#[derive(Clone, Copy, Debug)]
pub enum InputPin {
    K00 = 0x0,
    K01 = 0x1,
    K02 = 0x2,
    K03 = 0x3,
    K10 = 0x4,
    K11 = 0x5,
    K12 = 0x6,
    K13 = 0x7,
}

impl InputPin {
    pub fn from_u8(val: u8) -> Option<Self> {
        match val {
            0x0 => Some(InputPin::K00),
            0x1 => Some(InputPin::K01),
            0x2 => Some(InputPin::K02),
            0x3 => Some(InputPin::K03),
            0x4 => Some(InputPin::K10),
            0x5 => Some(InputPin::K11),
            0x6 => Some(InputPin::K12),
            0x7 => Some(InputPin::K13),
            _ => None,
        }
    }
}

#[derive(Debug)]
pub struct Inputs {
    pub ports: [u8; 2],
}

impl Inputs {
    pub fn new() -> Self {
        Self { ports: [0; 2] }
    }

    pub fn init(&mut self) {
        self.set_pin(InputPin::K00, true);
        self.set_pin(InputPin::K01, true);
        self.set_pin(InputPin::K02, true);
        self.set_pin(InputPin::K03, true);
    }

    pub fn set_pin(&mut self, pin: InputPin, value: bool) {
        self.ports[pin as usize & 0x4] = (self.ports[pin as usize & 0x4]
            & !(0x1 << (pin as usize & 0x3)))
            | ((value as u8) << (pin as usize & 0x3));
    }

    pub fn get_pin(&self, pin: InputPin) -> bool {
        (self.ports[pin as usize & 0x4] >> (pin as usize & 0x3) & 0x1) != 0
    }
}