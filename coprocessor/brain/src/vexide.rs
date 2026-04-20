use std::{
	io::{self, Write as _},
	rc::Rc,
	time::Instant,
};

use bytes::{BufMut, BytesMut};
use futures_util::pending;
use vexide::{prelude::SerialPort, smart::SmartPort, sync::Mutex};

use crate::requests::CoprocessorRequest;

#[derive(Clone)]
pub struct CoprocessorSmartPort {
	port: Rc<Mutex<SerialPort>>,
}

impl CoprocessorSmartPort {
	pub async fn new(port: SmartPort) -> Self {
		Self {
			port: Rc::new(Mutex::new(SerialPort::open(port, 921600).await)),
		}
	}

	pub fn send_request<R: CoprocessorRequest + 'static>(
		&self,
		request: R,
	) -> impl Future<Output = io::Result<R::Response>> + 'static {
		Self::send_request_with_port(self.port.clone(), request)
	}

	async fn send_request_with_port<R: CoprocessorRequest + 'static>(
		port_lock: Rc<Mutex<SerialPort>>,
		request: R,
	) -> Result<R::Response, io::Error> {
		// Scope that takes out a lock on the port
		let mut buf = {
			let mut port = port_lock.lock().await;

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

				pending!()
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
}
