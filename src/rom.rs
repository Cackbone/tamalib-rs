use crate::cpu::InstructionWithArgs;

pub fn load_rom(raw_rom: Vec<u8>) -> Vec<u16> {
    let mut rom = vec![];

    for i in (0..raw_rom.len()).step_by(2) {
        rom.push(raw_rom[i + 1] as u16 | ((raw_rom[i] as u16) << 8));
    }

    rom
}

#[allow(dead_code)]
pub fn print_rom(rom: Vec<u16>) {
    println!("{}", rom
                    .iter()
                    .map(|code| InstructionWithArgs::from_code(*code).unwrap().to_string())
                    .collect::<Vec<String>>()
                    .join("\n"))
}