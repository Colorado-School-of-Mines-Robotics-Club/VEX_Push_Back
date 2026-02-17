mod calibrate;
mod get_position;
mod get_velocity;
mod ping;
mod set_led;
mod set_offsets;
mod set_position;
mod set_scalars;

use core::time::Duration;

pub use calibrate::*;
pub use get_position::*;
pub use get_velocity::*;
pub use ping::*;
pub use set_led::*;
pub use set_offsets::*;
pub use set_position::*;
pub use set_scalars::*;

use bytes::Bytes;
use nom::{IResult, Parser};

pub trait CoprocessorRequest {
	const RESPONSE_SIZE: usize;
	const TIMEOUT: Duration = Duration::from_millis(100);

	type Response;

	fn serialize_request(&self) -> Bytes;
	fn parse_response(input: &[u8]) -> IResult<&[u8], Self::Response>;
}

fn parse_i16_triple(input: &[u8]) -> IResult<&[u8], (i16, i16, i16)> {
	(
		nom::number::complete::le_i16,
		nom::number::complete::le_i16,
		nom::number::complete::le_i16,
	)
		.parse(input)
}

fn assert_byte_tag(byte: u8) -> impl FnMut(&[u8]) -> IResult<&[u8], ()> {
	move |input| {
		nom::combinator::value((), nom::bytes::complete::tag::<&[u8], _, _>(&[byte])).parse(input)
	}
}
