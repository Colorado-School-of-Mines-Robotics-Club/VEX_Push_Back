use core::time::Duration;

use alloc::sync::Arc;
use bytes::{BufMut, BytesMut};
use vexide::{
    io::{self, Write, println},
    prelude::{SerialPort, SmartPort},
    sync::{Mutex, RwLock, RwLockReadGuard},
    task::{self, Task},
    time::{Instant, sleep},
};

use crate::requests::{
    CoprocessorRequest, GetPositionRequest, GetVelocityRequest, OtosPosition, OtosVelocity,
};

#[derive(Default)]
pub struct CoprocessorData {
    position: OtosPosition,
    velocity: OtosVelocity,
}

pub struct CoprocessorSmartPort {
    port: Arc<Mutex<SerialPort>>,
    latest_data: Arc<RwLock<CoprocessorData>>,
    _task: Task<!>,
}

impl CoprocessorSmartPort {
    pub async fn new(port: SmartPort) -> Self {
        let port = Arc::new(Mutex::new(SerialPort::open(port, 921600).await));
        let latest_data = Arc::new(RwLock::new(CoprocessorData::default()));
        Self {
            _task: task::spawn(Self::background_task(port.clone(), latest_data.clone())),
            latest_data,
            port,
        }
    }

    pub async fn get_data(&self) -> RwLockReadGuard<'_, CoprocessorData> {
        self.latest_data.read().await
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

        let encoded = cobs::encode_vec(&request.serialize_request());
        port.write_all(&encoded)?;
        port.write(&[0x00])?;

        let timeout = Instant::now() + R::TIMEOUT;
        let mut buf =
            BytesMut::with_capacity(const { cobs::max_encoding_length(R::RESPONSE_SIZE) + 1 });

        loop {
            if Instant::now() > timeout {
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

    async fn background_task(
        port: Arc<Mutex<SerialPort>>,
        latest_data: Arc<RwLock<CoprocessorData>>,
    ) -> ! {
        loop {
            match Self::send_request_with_port(port.clone(), GetPositionRequest).await {
                Ok(position) => {
                    latest_data.write().await.position = position;
                }
                Err(e) => println!("Error reading position from coprocessor: {e:?}"),
            };

            match Self::send_request_with_port(port.clone(), GetVelocityRequest).await {
                Ok(velocity) => {
                    latest_data.write().await.velocity = velocity;
                }
                Err(e) => println!("Error reading velocity from coprocessor: {e:?}"),
            };

            sleep(Duration::from_millis(5)).await
        }
    }
}
