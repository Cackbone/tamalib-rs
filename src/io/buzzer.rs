pub const BUZZER_FREQUENCIES: [usize; 8] = [40960, 32768, 27307, 23406, 20480, 16384, 13653, 11703];

pub trait Buzzer {
    // Set the frequency of the buzzer
    fn set_frequency(&mut self, freq: usize);

    // Enable the buzzer
    fn play(&mut self, value: bool);
}
