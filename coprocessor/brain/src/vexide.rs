use core::time::Duration;

use alloc::sync::Arc;
use bytes::{BufMut, BytesMut};
use vexide::{
    io::{self, Write},
    prelude::{SerialPort, SmartPort},
    sync::Mutex,
    task::{self, Task},
    time::{Instant, sleep},
};

use crate::requests::{CoprocessorRequest, PingRequest};

pub struct CoprocessorSmartPort {
    port: Arc<Mutex<SerialPort>>,
    _task: Task<!>,
}

impl CoprocessorSmartPort {
    pub async fn new(port: SmartPort) -> Self {
        let port = Arc::new(Mutex::new(SerialPort::open(port, 921600).await));
        Self {
            _task: task::spawn(Self::background_task(port.clone())),
            port,
        }
    }

    pub async fn send_request<R: CoprocessorRequest>(
        &self,
        request: R,
    ) -> Result<R::Response, io::Error> {
        Self::send_request_with_port(self.port.clone(), request).await
    }

    async fn send_request_with_port<R: CoprocessorRequest>(
        port: Arc<Mutex<SerialPort>>,
        request: R,
    ) -> Result<R::Response, io::Error> {
        let mut port = port.lock().await;

        port.write_all(&request.serialize_request())?;

        let time = Instant::now();
        let mut buf =
            BytesMut::with_capacity(const { cobs::max_encoding_length(R::RESPONSE_SIZE) + 1 });

        loop {
            if Instant::now() > time {
                return Err(io::Error::new(
                    io::ErrorKind::TimedOut,
                    "waiting for full response timed out",
                ));
            } else if buf.capacity() == 0 {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidData,
                    "recieved data was too long",
                ));
            }

            if let Some(byte) = port.read_byte() {
                buf.put_u8(byte);

                if byte == b'\0' {
                    break;
                }
            }

            sleep(Duration::from_millis(5)).await;
        }

        let Ok(len) = cobs::decode_in_place(&mut buf[..]) else {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "recieved data was unable to be COBS-decoded",
            ));
        };
        buf.truncate(len);
        let buf = buf.freeze();

        R::parse_response(&buf).map(|(_, d)| d).map_err(|_| {
            io::Error::new(
                io::ErrorKind::InvalidData,
                "decoded data was unable to be parsed correctly",
            )
        })
    }

    async fn background_task(port: Arc<Mutex<SerialPort>>) -> ! {
        loop {
            let _ = Self::send_request_with_port(port.clone(), PingRequest).await;

            sleep(Duration::from_millis(100)).await
        }
    }
}
