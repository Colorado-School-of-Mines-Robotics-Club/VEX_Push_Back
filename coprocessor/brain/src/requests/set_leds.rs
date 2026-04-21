use bytes::Bytes;
use nom::Parser as _;

use crate::requests::{CoprocessorRequest, assert_byte_tag};

pub struct SetLedsRequest<const COLOR: u32>;

pub const RAINBOW_STATIC: u32 = 0xFF000001;
pub const RAINBOW_ROTATE: u32 = 0xFF000002;

impl<const COLOR: u32> CoprocessorRequest for SetLedsRequest<COLOR> {
	const RESPONSE_SIZE: usize = size_of::<u8>();

	type Response = ();

	fn serialize_request(&self) -> Bytes {
		Bytes::from_static(
			&const {
				let mut arr = [0u8; 1 + size_of::<u32>()];
				arr[0] = b'l';
				arr.split_at_mut(1).1.copy_from_slice(&COLOR.to_le_bytes());

				arr
			},
		)
	}

	fn parse_response(input: &[u8]) -> nom::IResult<&[u8], Self::Response> {
		nom::combinator::all_consuming(assert_byte_tag(b'd')).parse(input)
	}
}
