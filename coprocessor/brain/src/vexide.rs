use std::{
    any::TypeId,
    io::{self, Write as _},
    sync::Arc,
    time::{Duration, Instant},
};

use bytes::{BufMut, BytesMut};
use shrewnit::{Length, Radians};
use vexide::{
    prelude::SerialPort,
    smart::SmartPort,
    sync::Mutex,
    task::{self, Task},
    time::sleep,
};

use crate::requests::{
    CoprocessorRequest, GetPositionRequest, GetVelocityRequest, OtosPosition, OtosVelocity,
    PingRequest,
};

macro_rules! make_coprocessor_data {
    ($(pub $name:ident: $type:ty),+) => {
        pub struct CoprocessorData {
            $(pub $name: ::triple_buffer::Output<$type>),+
        }

        pub struct CoprocessorDataSink {
            $(pub $name: ::triple_buffer::Input<$type>),+
        }

        impl CoprocessorData {
            pub fn new() -> (CoprocessorDataSink, Self) {
                $(let $name = ::triple_buffer::TripleBuffer::new(&Default::default()).split();)+

                (
                    CoprocessorDataSink {
                        $($name: $name.0),+
                    },
                    CoprocessorData {
                        $($name: $name.1),+
                    }
                )
            }
        }
    };
}

make_coprocessor_data!(
    pub forward_travel: Length<f64>,
    pub position: OtosPosition,
    pub velocity: OtosVelocity
);

pub struct CoprocessorSmartPort {
    port: Arc<Mutex<SerialPort>>,
    _task: Task<!>,
}

impl CoprocessorSmartPort {
    pub async fn new(port: SmartPort) -> (Self, CoprocessorData) {
        let port = Arc::new(Mutex::new(SerialPort::open(port, 115200).await));
        let (latest_data_sink, latest_data) = CoprocessorData::new();
        (
            Self {
                _task: task::spawn(Self::background_task(port.clone(), latest_data_sink)),
                port,
            },
            latest_data,
        )
    }

    pub fn send_request<R: CoprocessorRequest + 'static>(
        &self,
        request: R,
    ) -> impl Future<Output = io::Result<R::Response>> + 'static {
        Self::send_request_with_port(self.port.clone(), request)
    }

    async fn send_request_with_port<R: CoprocessorRequest + 'static>(
        port_lock: Arc<Mutex<SerialPort>>,
        request: R,
    ) -> Result<R::Response, io::Error> {
        // Scope that takes out a lock on the port
        let mut buf = {
            // The vexide mutex gets deadlocked for some reason when not using try_lock
            let mut port = port_lock.lock().await;
            // let mut port;
            // loop {
            //     match port_lock.try_lock() {
            //         Some(lock) => {
            //             port = lock;
            //             break;
            //         }
            //         None => {
            //             sleep(Duration::from_millis(0)).await;
            //         }
            //     }
            // }

            let encoded = cobs::encode_vec(&request.serialize_request());
            port.write_all(&encoded)?;
            port.write_all(&[0x00])?;

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

        R::parse_response(&buf).map(|(_, d)| d).map_err(|e| {
            // dbg!(e);
            io::Error::new(
                io::ErrorKind::InvalidData,
                "decoded data was unable to be parsed correctly",
            )
        })
    }

    async fn background_task(
        port: Arc<Mutex<SerialPort>>,
        mut latest_data_sink: CoprocessorDataSink,
    ) -> ! {
        let mut last_position = OtosPosition::default();
        let mut forward_travel = Length::default();
        let mut i: usize = 0;
        loop {
            // println!("Sending position request...");
            match Self::send_request_with_port(port.clone(), GetPositionRequest).await {
                Ok(position) => {
                    // Update the position through the sink
                    latest_data_sink.position.write(position);

                    // Calculate the direction in which we moved, and the total distance that we moved
                    let diff = (position.x - last_position.x, position.y - last_position.y);
                    let movement_angle =
                        f64::atan2(diff.1.canonical(), diff.0.canonical()) * Radians;
                    let distance = Length::from_canonical(
                        (diff.0.canonical().powi(2) + diff.1.canonical().powi(2)).sqrt(),
                    );

                    // Use the angle of movement and the heading to find how much we moved in the direction
                    // of our current heading (the travel, ignoring any non-"forward" movement)
                    let forward_travel_delta =
                        distance * f64::cos((movement_angle - position.heading).to::<Radians>());

                    // Update state and update new forward travel through the sink
                    last_position = position;
                    forward_travel += forward_travel_delta;
                    latest_data_sink.forward_travel.write(forward_travel);
                }
                Err(_e) => (), // TODO: error state indication?
            };

            // println!("Sending velocity request...");
            match Self::send_request_with_port(port.clone(), GetVelocityRequest).await {
                Ok(velocity) => {
                    latest_data_sink.velocity.write(velocity);
                }
                Err(_e) => (), // TODO: error state indication?
            };

            i = i.wrapping_add(3);
            sleep(Duration::from_millis(3)).await
        }
    }
}
