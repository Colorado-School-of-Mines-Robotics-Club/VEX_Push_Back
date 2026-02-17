use bytes::{BufMut, Bytes, BytesMut};
use nom::Parser as _;

use crate::{
	OtosAngle, OtosLength,
	requests::{CoprocessorRequest, OtosPosition, assert_byte_tag},
};
pub struct SetPositionRequest(pub OtosPosition);

impl CoprocessorRequest for SetPositionRequest {
	const RESPONSE_SIZE: usize = size_of::<char>();

	type Response = ();

	fn serialize_request(&self) -> Bytes {
		let mut buf = BytesMut::with_capacity(1 + size_of::<i16>() * 3);

		buf.put_u8(b'P');
		buf.put_i16_le(self.0.x.to::<OtosLength>() as i16);
		buf.put_i16_le(self.0.y.to::<OtosLength>() as i16);
		buf.put_i16_le(self.0.heading.to::<OtosAngle>() as i16);

		buf.freeze()
	}

	fn parse_response(input: &[u8]) -> nom::IResult<&[u8], Self::Response> {
		nom::combinator::all_consuming(assert_byte_tag(b'd')).parse(input)
	}
}
