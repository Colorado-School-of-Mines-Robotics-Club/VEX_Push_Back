use core::cell::RefCell;

use alloc::{boxed::Box, sync::Arc};
use vexide::{
    io,
    sync::Mutex,
    task::{Task, spawn},
};

const BUFFER_SIZE: usize = 2 * 1024 /* 1 KiB */;

#[repr(u8)]
enum BrainCode {
    Keepalive = 0x01,
    Recalibrate = 0x02,
}

#[derive(Debug, Default)]
pub struct OtosPosition {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

pub struct OtosSensor {
    latest_data: Arc<Mutex<OtosPosition>>,
}

impl<S: io::Read + io::Write> OtosSensor {
    pub fn new(serial: S) -> Self {
        let sensor = Self {
            latest_data: Arc::new(Mutex::new(OtosPosition::default())),
        };

        spawn(async move {
            let mut buffer = [0u8; BUFFER_SIZE];

            loop {
                let Ok(byte) = io.read(&mut buffer) else {
                    continue;
                };
            }
        });

        sensor
    }
}
