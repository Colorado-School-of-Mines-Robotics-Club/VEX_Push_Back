use std::time::Duration;

use bytes::Bytes;
use nom::Parser as _;

use crate::requests::CoprocessorRequest;

pub struct PingRequest;

impl PingRequest {
	const EXPECTED_PICO_SHA256_HEX: &str = env!("PICO_MICROPYTHON_SHA256");
	pub const EXPECTED_PICO_SHA256: [u8; 32] = {
		let mut i = 0;
		let mut result = [0u8; 32];
		let hex = Self::EXPECTED_PICO_SHA256_HEX;
		while i < hex.len() {
			let Ok(byte) = u8::from_str_radix(hex.split_at(i).1.split_at(2).0, 16) else {
				panic!();
			};
			result[i / 2] = byte;
			i += 2;
		}

		result
	};

	pub fn verify_hash(actual: [u8; 32]) -> bool {
		actual == Self::EXPECTED_PICO_SHA256
	}
}

impl CoprocessorRequest for PingRequest {
	const RESPONSE_SIZE: usize = size_of::<Self::Response>();
	const TIMEOUT: Duration = Duration::from_millis(500);

	type Response = [u8; 32];

	fn serialize_request(&self) -> Bytes {
		Bytes::from_static(b"a")
	}

	fn parse_response(input: &[u8]) -> nom::IResult<&[u8], Self::Response> {
		nom::combinator::all_consuming(nom::combinator::map_res(
			nom::bytes::complete::take::<usize, _, _>(32 /* SHA256 hash */),
			|bytes: &[u8]| bytes.try_into(),
		))
		.parse(input)
	}
}
