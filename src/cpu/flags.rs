#[repr(u8)]
pub enum Flag {
    C = (0x1 << 0),
    Z = (0x1 << 1),
    D = (0x1 << 2),
    I = (0x1 << 3),
}

#[derive(Default, Debug)]
pub struct Flags(pub u8);

impl Flags {
    pub fn get(&self, flag: Flag) -> bool {
        self.0 & (flag as u8) != 0
    }

    pub fn set(&mut self, flag: Flag, value: bool) {
        if value {
            self.0 |= flag as u8;
        } else {
            self.0 &= !(flag as u8);
        }
    }
}