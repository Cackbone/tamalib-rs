use rodio::{OutputStream, Sink, source::SineWave, Source};
use std::sync::{Arc, Mutex};
use tamalib_rs::Buzzer;


pub struct TamaBuzzer {
    freq: Arc<Mutex<usize>>,
    _stream: Option<OutputStream>,
    stream_handle: Option<rodio::OutputStreamHandle>,
}

impl TamaBuzzer {
    pub fn new() -> Self {
        match OutputStream::try_default() {
            Ok((stream, stream_handle)) => Self {
                freq: Arc::new(Mutex::new(1000)),
                _stream: Some(stream),
                stream_handle: Some(stream_handle),
            },
            Err(e) => {
                eprintln!("Audio device not available: {e}");
                Self {
                    freq: Arc::new(Mutex::new(1000)),
                    _stream: None,
                    stream_handle: None,
                }
            }
        }
    }
}

impl Buzzer for TamaBuzzer {
    fn set_frequency(&mut self, freq: usize) {
        let mut f = self.freq.lock().unwrap();
        *f = freq;
    }
    fn play(&mut self, value: bool) {
        if value {
            if let Some(ref stream_handle) = self.stream_handle {
                if let Ok(sink) = Sink::try_new(stream_handle) {
                    let source = SineWave::new(*self.freq.lock().unwrap() as f32)
                        .take_duration(std::time::Duration::from_millis(100))
                        .amplify(0.20);
                    sink.append(source);
                    sink.detach();
                }
            } else {
                eprintln!("Audio device not available: cannot play buzzer.");
            }
        }
    }
}