pub const SCREEN_WIDTH: usize = 32;
pub const SCREEN_HEIGHT: usize = 16;
pub const ICONS_COUNT: usize = 8;
pub const SEG_POS: [u8; 51] = [
    0, 1, 2, 3, 4, 5, 6, 7, 32, 8, 9, 10, 11, 12, 13, 14, 15, 33, 34, 35, 31, 30, 29, 28, 27, 26,
    25, 24, 36, 23, 22, 21, 20, 19, 18, 17, 16, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49,
    50,
];

pub trait Screen {
    // Update the screen
    fn update(&mut self);

    // Set a pixel on the screen at the given coordinates
    fn set_pixel(&mut self, x: usize, y: usize, value: bool);

    // Turn on or off an icon on the screen
    fn set_icon(&mut self, icon: usize, value: bool);
}
