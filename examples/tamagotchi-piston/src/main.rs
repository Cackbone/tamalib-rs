mod graphics;
mod buzzer;
mod clock;
mod logger;

use piston_window::*;
use tamalib_rs::{Tamagotchi, LogLevel};
use graphics::{Ui, PistonScreen, WIDTH, HEIGHT, SCALE, BUTTON_AREA_HEIGHT, ICON_ROW_HEIGHT};
use buzzer::TamaBuzzer;
use clock::SystemClock;
use logger::TamaLogger;

use piston_window::PistonWindow;
use std::rc::Rc;
use std::cell::RefCell;
use clap::Parser;


const SPEED: [u8; 3] = [1, 5, 0]; // x1, x5, MAX

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Path to the ROM file
    #[arg(short, long, value_name = "ROM_PATH", default_value = "rom.bin")]
    rom: String,

    /// Enable CPU logging
    #[arg(short, long)]
    cpu: bool,

    /// Enable memory logging
    #[arg(short, long)]
    memory: bool,
}

fn main() {
    let args = Args::parse();
    let rom_path = &args.rom;

    let mut window: PistonWindow = WindowSettings::new(
        format!("Tamagotchi ({}x{})", WIDTH * SCALE, HEIGHT * SCALE + BUTTON_AREA_HEIGHT + ICON_ROW_HEIGHT),
        [WIDTH * SCALE, HEIGHT * SCALE + BUTTON_AREA_HEIGHT + ICON_ROW_HEIGHT],
    )
    .resizable(false)
    .exit_on_esc(true)
    .build()
    .unwrap();

    let icon_texture: G2dTexture = Texture::from_path(
        &mut window.create_texture_context(),
        "./examples/tamagotchi-piston/res/p1-icons.png",
        Flip::None,
        &TextureSettings::new()
    ).expect("Failed to load icons");

    let rom = if std::fs::read(rom_path).is_ok() {
        std::fs::read(rom_path).unwrap()
    } else {
        eprintln!("Failed to load ROM from path: {}", rom_path);
        return;
    };
    let screen = Rc::new(RefCell::new(PistonScreen::new()));
    let buzzer = Box::new(TamaBuzzer::new());
    let clock = Box::new(SystemClock);
    let mut logger = Box::new(TamaLogger::new());

    if args.cpu {
        logger.set_log_level(LogLevel::Cpu, true);
    }
    if args.memory {
        logger.set_log_level(LogLevel::Memory, true);
    }

    let mut tama = Tamagotchi::builder()
        .rom(rom)
        .screen(screen.clone())
        .buzzer(buzzer)
        .system_clock(clock)
        .logger(logger)
        .build();
    let mut ui = Ui::new(icon_texture);
    let mut speed = 0;

    /*loop {
        tama.run_step();
    }*/
    while let Some(e) = window.next() {
        loop {
            tama.run_step();

            ui.on_key_toggle(&e, Key::S, || {
                speed = (speed + 1) % SPEED.len();
                tama.set_speed(SPEED[speed]);
            });

            let s = screen.borrow();
            
            ui.handle_events(&e);
            ui.icon_states = s.icon_states;
            use tamalib_rs::Button;
            for (i, &state) in ui.button_states.iter().enumerate() {
                if state != ui.prev_button_states[i] {
                    let button = match i {
                        0 => Button::LEFT,
                        1 => Button::MIDDLE,
                        2 => Button::RIGHT,
                        _ => continue,
                    };
                    tama.io.set_button(button, state);
                }
                ui.prev_button_states[i] = state;
            }

            if s.request_redraw {
                break;
            }
        }
        if let Some(_r) = e.render_args() {
            let mut s = screen.borrow_mut();
            if s.request_redraw {
                s.redraw(&mut window, &ui);
            }
        }
    }
} 