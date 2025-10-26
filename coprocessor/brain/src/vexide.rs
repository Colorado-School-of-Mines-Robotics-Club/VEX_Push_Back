use core::{
    ops::{Add, Deref},
    sync::atomic::Ordering,
    time::Duration,
};

use alloc::sync::Arc;
use atomic::Atomic;
use bytemuck::NoUninit;
use bytes::{BufMut, BytesMut};
use shrewnit::{Length, Radians};
use vexide::{
    float::Float,
    io::{self, Write},
    prelude::{SerialPort, SmartPort},
    sync::Mutex,
    task::{self, Task},
    time::{Instant, sleep},
};

use crate::requests::{
    CoprocessorRequest, GetPositionRequest, GetVelocityRequest, OtosPosition, OtosVelocity,
};

#[derive(Default, Copy, Clone)]
pub struct OtosForwardTravel(Length<f64>);
unsafe impl NoUninit for OtosForwardTravel {}

impl Deref for OtosForwardTravel {
    type Target = Length<f64>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl Add<Length<f64>> for OtosForwardTravel {
    type Output = Self;

    fn add(self, rhs: Length<f64>) -> Self::Output {
        Self(self.0 + rhs)
    }
}

#[derive(Default)]
pub struct CoprocessorData {
    pub forward_travel: Atomic<OtosForwardTravel>,
    pub position: Atomic<OtosPosition>,
    pub velocity: Atomic<OtosVelocity>,
}

pub struct CoprocessorSmartPort {
    port: Arc<Mutex<SerialPort>>,
    latest_data: Arc<CoprocessorData>,
    _task: Task<!>,
}

impl CoprocessorSmartPort {
    pub async fn new(port: SmartPort) -> Self {
        let port = Arc::new(Mutex::new(SerialPort::open(port, 921600).await));
        let latest_data = Arc::new(CoprocessorData::default());
        Self {
            _task: task::spawn(Self::background_task(port.clone(), latest_data.clone())),
            latest_data,
            port,
        }
    }

    /// Returns a reference that points to the latest data from the sensor
    pub fn latest_data(&self) -> Arc<CoprocessorData> {
        self.latest_data.clone()
    }

    pub async fn send_request<R: CoprocessorRequest>(
        &self,
        request: R,
    ) -> Result<R::Response, io::Error> {
        Self::send_request_with_port(self.port.clone(), request).await
    }

    async fn send_request_with_port<R: CoprocessorRequest>(
        port_lock: Arc<Mutex<SerialPort>>,
        request: R,
    ) -> Result<R::Response, io::Error> {
        // Scope that takes out a lock on the port
        let mut buf = {
            let mut port = port_lock.lock().await;

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

            buf
        };

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

    async fn background_task(port: Arc<Mutex<SerialPort>>, latest_data: Arc<CoprocessorData>) -> ! {
        loop {
            match Self::send_request_with_port(port.clone(), GetPositionRequest).await {
                Ok(position) => {
                    let old_position = latest_data.position.swap(position, Ordering::Relaxed);

                    _ = latest_data.forward_travel.fetch_update(
                        Ordering::Relaxed,
                        Ordering::Relaxed,
                        |forward_travel| {
                            let diff = (position.x - old_position.x, position.y - old_position.y);

                            // Calculate the direction in which we moved, and the total distance that we moved
                            let movement_angle =
                                f64::atan2(diff.1.canonical(), diff.0.canonical()) * Radians;
                            let distance = Length::from_canonical(
                                (diff.0.canonical().powi(2) + diff.1.canonical().powi(2)).sqrt(),
                            );

                            // Use the angle of movement and the heading to find how much we moved in the direction
                            // of our current heading (the travel, ignoring any non-"forward" movement)
                            //
                            // TODO: average old and new heading for the heading aspect? is that helpful?
                            let forward_travel_delta = distance
                                * f64::cos((movement_angle - position.heading).to::<Radians>());

                            Some(forward_travel + forward_travel_delta)
                        },
                    );
                }
                Err(_e) => (), // TODO: error state indication?
            };

            match Self::send_request_with_port(port.clone(), GetVelocityRequest).await {
                Ok(velocity) => {
                    latest_data.velocity.store(velocity, Ordering::Relaxed);
                }
                Err(_e) => (), // TODO: error state indication?
            };

            sleep(Duration::from_millis(5)).await
        }
    }
}
