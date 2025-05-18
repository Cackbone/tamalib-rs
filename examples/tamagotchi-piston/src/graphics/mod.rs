mod ui;

use piston_window::*;
use tamalib_rs::Screen;


pub use ui::Ui;

pub const WIDTH: u32 = 32;
pub const HEIGHT: u32 = 16;
pub const SCALE: u32 = 10;

pub const BUTTON_AREA_HEIGHT: u32 = 8 * SCALE;
const BUTTON_RADIUS: u32 = 2;

pub const ICON_ROW_HEIGHT: u32 = 8 * SCALE;
const ICON_PIXEL_SIZE: u32 = 4;


pub struct PistonScreen {
    pub framebuffer: Vec<u8>,
    pub request_redraw: bool,
    pub icon_states: [bool; 8]
}

impl PistonScreen {
    pub fn new() -> Self {
        Self {
            framebuffer: vec![0; (WIDTH * HEIGHT) as usize],
            request_redraw: true,
            icon_states: [false; 8]
        }
    }

    pub fn redraw(&mut self, window: &mut PistonWindow, ui: &Ui) {
        let window_height = ICON_ROW_HEIGHT * 2 + HEIGHT * SCALE + BUTTON_AREA_HEIGHT;
        window.draw_2d(&Event::Loop(Loop::Render(RenderArgs {
            ext_dt: 0.0,
            window_size: [(WIDTH * SCALE) as f64, window_height as f64],
            draw_size: [(WIDTH * SCALE) as u32, window_height as u32],
        })), |c, g, _| {
            clear([1.0; 4], g);
            // Draw pixels (shifted down by ICON_ROW_HEIGHT)
            for y in 0..HEIGHT {
                for x in 0..WIDTH {
                    let v = self.framebuffer[(y * WIDTH + x) as usize];
                    let color = if v != 0 {
                        [0.0, 0.0, 0.0, 1.0] // on: black
                    } else {
                        [60.0/255.0, 65.0/255.0, 44.0/255.0, 1.0] // off: #3C412C
                    };
                    rectangle(
                        color,
                        [
                            (x * SCALE) as f64,
                            (y * SCALE) as f64 + ICON_ROW_HEIGHT as f64,
                            SCALE as f64,
                            SCALE as f64,
                        ],
                        c.transform,
                        g,
                    );
                }
            }
            // Draw grid (shifted down by ICON_ROW_HEIGHT)
            for x in 0..=WIDTH {
                let xf = (x * SCALE) as f64;
                line(
                    [0.157, 0.172, 0.118, 1.0], // darker than off pixel
                    0.5,
                    [xf, ICON_ROW_HEIGHT as f64, xf, (HEIGHT * SCALE) as f64 + ICON_ROW_HEIGHT as f64],
                    c.transform,
                    g,
                );
            }
            for y in 0..=HEIGHT {
                let yf = (y * SCALE) as f64 + ICON_ROW_HEIGHT as f64;
                line(
                    [0.157, 0.172, 0.118, 1.0], // darker than off pixel
                    1.0,
                    [0.0, yf, (WIDTH * SCALE) as f64, yf],
                    c.transform,
                    g,
                );
            }
            
            // Draw UI (buttons and icons)
            ui.draw(c, g);
        });
        self.request_redraw = false;
    }
}

impl Screen for PistonScreen {
    fn update(&mut self) {
        self.request_redraw = true;
    }

    fn set_pixel(&mut self, x: usize, y: usize, value: bool) {
        if x < WIDTH as usize && y < HEIGHT as usize {
            self.framebuffer[y * WIDTH as usize + x] = value as u8;
        }
    }
    fn set_icon(&mut self, icon: usize, value: bool) {
        if icon < 8 {
            self.icon_states[icon] = value;
        }
    }
}