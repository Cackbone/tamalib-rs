use piston_window::*;
use super::{WIDTH, HEIGHT, SCALE, BUTTON_AREA_HEIGHT, ICON_ROW_HEIGHT, BUTTON_RADIUS, ICON_PIXEL_SIZE};
use std::collections::HashSet;


pub struct Ui {
    spacing: f64,
    pub button_states: [bool; 3],
    pub prev_button_states: [bool; 3],
    last_mouse_pos: [f64; 2],
    pub icon_states: [bool; 8],
    icon_texture: G2dTexture,
    pressed_keys: HashSet<Key>,
}

impl Ui {
    pub fn new(icon_texture: G2dTexture) -> Self {
        let spacing = (WIDTH * SCALE) as f64 / 4.0;
        Self {
            spacing,
            button_states: [false; 3],
            prev_button_states: [false; 3],
            last_mouse_pos: [0.0, 0.0],
            icon_states: [false; 8],
            icon_texture,
            pressed_keys: HashSet::new(),
        }
    }

    pub fn draw(&self, c: Context, g: &mut G2d) {
        // Draw top icons row (with background)
        self.draw_icons_row(0, 4, 0.0, c, g);
        // Draw button area background (at the very bottom)
        let window_height = ICON_ROW_HEIGHT * 2 + HEIGHT * SCALE + BUTTON_AREA_HEIGHT;
        rectangle(
            [1.0, 1.0, 1.0, 1.0], // white
            [
                0.0,
                (window_height - BUTTON_AREA_HEIGHT) as f64,
                (WIDTH * SCALE) as f64,
                BUTTON_AREA_HEIGHT as f64,
            ],
            c.transform,
            g,
        );
        let y = (window_height - BUTTON_AREA_HEIGHT - ICON_ROW_HEIGHT) as f64; 
        self.draw_icons_row(4, 8, y, c, g);
        // Draw 3 red circles, darker if pressed (in the button area)
        for i in 0..3 {
            let button_x = self.spacing * (i as f64 + 1.0);
            let color = if self.button_states[i] {
                [0.6, 0.0, 0.0, 1.0] // darker red
            } else {
                [1.0, 0.0, 0.0, 1.0] // normal red
            };
            ellipse(
                color,
                [
                    button_x - BUTTON_RADIUS as f64 * SCALE as f64,
                    (window_height - BUTTON_AREA_HEIGHT) as f64 + BUTTON_AREA_HEIGHT as f64 / 2.0 - BUTTON_RADIUS as f64 * SCALE as f64,
                    BUTTON_RADIUS as f64 * 2.0 * SCALE as f64,
                    BUTTON_RADIUS as f64 * 2.0 * SCALE as f64,
                ],
                c.transform,
                g,
            );
        }
    }

    fn draw_icons_row(&self, start: usize, end: usize, y: f64, c: Context, g: &mut G2d) {
        let icon_width = ICON_PIXEL_SIZE as f64 * SCALE as f64;
        let total_icons = end - start;
        let total_width = (icon_width * total_icons as f64) + (icon_width * (total_icons as f64 - 1.0) * 0.25);
        let x0 = ((WIDTH * SCALE) as f64 - total_width) / 2.0;
        // Draw row background
        rectangle(
            [60.0/255.0, 65.0/255.0, 44.0/255.0, 1.0],
            [
                0.0,
                y.floor(),
                (WIDTH * SCALE) as f64,
                ICON_ROW_HEIGHT as f64,
            ],
            c.transform,
            g,
        );
        for i in start..end {
            let x = x0 + (i - start) as f64 * icon_width * 1.25;
            self.draw_icon(i, x, y + ICON_ROW_HEIGHT as f64 / 2.0 - 32.0, c, g);
        }
    }

    fn draw_icon(&self, icon_index: usize, x: f64, y: f64, c: Context, g: &mut G2d) {
        let state = self.icon_states.get(icon_index).copied().unwrap_or(false);
        let color = if state {
            [0.0, 0.0, 0.0, 1.0] // on: black
        } else {
            [0.157, 0.172, 0.118, 1.0]
        };
        let icon_src_w = 64.0;
        let icon_src_h = 64.0;
        let tex_h = self.icon_texture.get_height() as f64;
        let src_rect = [
            (icon_index as f64 % 4.0) * icon_src_w,
            if icon_index < 4 { 0.0 } else { tex_h - icon_src_h },
            icon_src_w,
            icon_src_h,
        ];

        Image::new_color(color)
            .src_rect(src_rect)
            .draw(
                &self.icon_texture,
                &c.draw_state,
                c.transform.trans(x, y),
                g,
            );
    }

    pub fn handle_events(&mut self, e: &Event) {
        if let Some(pos) = e.mouse_cursor_args() {
            self.last_mouse_pos = pos;
        }

        if let Some(Button::Mouse(MouseButton::Left)) = e.press_args() {
            for i in 0..3 {
                let button_x = self.spacing * (i as f64 + 1.0);
                let button_y = (ICON_ROW_HEIGHT + HEIGHT * SCALE) as f64 + BUTTON_AREA_HEIGHT as f64 / 2.0;
                let dx = self.last_mouse_pos[0] - button_x;
                let dy = self.last_mouse_pos[1] - button_y;
                if (dx * dx + dy * dy).sqrt() < (BUTTON_RADIUS as f64 * SCALE as f64) {
                    self.button_states[i] = true;
                }
            }
        } else if let Some(Button::Mouse(MouseButton::Left)) = e.release_args() {
            self.button_states = [false; 3];
        }
    }


    pub fn on_key_toggle(&mut self, e: &Event, key: Key, mut cb: impl FnMut()) {
        if let Some(Button::Keyboard(k)) = e.press_args() {
            if k == key {
                self.pressed_keys.insert(k);
            }
        } else if let Some(Button::Keyboard(k)) = e.release_args() {
            if k == key && self.pressed_keys.contains(&key) {
                cb();
                self.pressed_keys.remove(&k);
            }
        }
    }
}