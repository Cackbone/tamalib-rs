# tamalib-rs

<pre>
          ..'':
         :..''
      _.-|-._
    .'   |   '.
  /'           '.
 / \.---------./ \
/  |           |  \
|  |  TamaLib  |  |
|- |           | -|
|  |           |  |
 \ /`---------'\ /
  '\  o  o  o  /'
    '-.__|__.-'
</pre>

A simple, headless emulator library for the Tamagotchi P1 virtual pet, written in Rust.

Thanks to the already existing [tamalib project](https://github.com/jcrona/tamalib/) that helped me a lot on the CPU implementation.


[![Crates.io](https://img.shields.io/crates/v/tamalib)](https://crates.io/crates/tamalib)

---

## Features

- **CPU Emulation**: Emulates the Epson E0C6S46 CPU used in the original Tamagotchi P1.
- **Screen & Buzzer**: Abstract traits for screen and buzzer, allowing integration with any backend.
- **Button Input**: Simulate button presses (TAP, LEFT, MIDDLE, RIGHT).
- **Event-driven IO**: Uses an event bus for IO interactions.
- **Customizable Logging**: Plug in your own logger for debugging or tracing.

---

## Getting Started

### Add to your project

```toml
[dependencies]
tamalib = "0.1.0"
```

### Example Usage

```rust
use tamalib::{Tamagotchi, TamagotchiBuilder, Button, Screen, Buzzer, Clock, Logger, LogLevel};
use std::rc::Rc;
use std::cell::RefCell;
use std::fs;

// Implement the required traits for your platform:
struct DummyScreen;
impl Screen for DummyScreen {
    fn update(&mut self) {}
    fn set_pixel(&mut self, _x: usize, _y: usize, _value: bool) {}
    fn set_icon(&mut self, _icon: usize, _value: bool) {}
}

struct DummyBuzzer;
impl Buzzer for DummyBuzzer {
    fn set_frequency(&mut self, _freq: usize) {}
    fn play(&mut self, _value: bool) {}
}

struct DummyClock;
impl Clock for DummyClock {
    fn now(&self) -> usize { 0 }
}

struct DummyLogger;
impl Logger for DummyLogger {
    fn log(&self, _level: LogLevel, _message: &str) {}
    fn log_enabled(&self, _level: LogLevel) -> bool { true }
}

fn main() {
    let rom = fs::read("path/to/rom.bin").expect("Failed to read ROM file");
    let screen = Rc::new(RefCell::new(DummyScreen));
    let buzzer = Box::new(DummyBuzzer);
    let clock = Box::new(DummyClock);
    let logger = Box::new(DummyLogger);

    let mut tama = Tamagotchi::new(rom, screen, buzzer, clock, logger);

    // Run a single emulation step
    // The mainloop is not implemented inside the lib because it can create issues with some platforms like WASM
    tama.run_step();
}
```

---

## Library Structure

- **CPU**: Emulates the Tamagotchi's CPU, memory, and instruction set.
- **IO**: Abstracts screen, buzzer, and button input.
- **ROM**: Loading and decoding of Tamagotchi ROMs.
- **Logger**: Customizable logging interface.
- **Event Bus**: For decoupled IO event handling.

---

## Traits

You must implement the following traits for your platform:

- `Screen`: For display output.
- `Buzzer`: For sound output.
- `Clock`: For timing.
- `Logger`: For logging/debugging.

---

## License

See [LICENSE](LICENSE).

---

## Links

- [GitHub](https://github.com/Cackbone/tamalib-rs)
- [Crates.io](https://crates.io/crates/tamalib) 