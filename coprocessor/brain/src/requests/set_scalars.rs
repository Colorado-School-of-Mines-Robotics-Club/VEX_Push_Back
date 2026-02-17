use bytes::{BufMut, Bytes, BytesMut};
use nom::Parser as _;

use crate::requests::{CoprocessorRequest, assert_byte_tag};

pub struct OtosScalingFactor {
	inner: i8,
}

impl OtosScalingFactor {
	pub const fn parse(factor: f64) -> Self {
		assert!(factor <= 1.127, "percent value too large");
		assert!(factor >= 0.872, "percent value too small");

		Self {
			inner: ((factor - 1_f64) / 0.1 * 100_f64) as i8,
		}
	}
}

pub struct SetScalarsRequest {
	pub linear: OtosScalingFactor,
	pub angular: OtosScalingFactor,
}

impl CoprocessorRequest for SetScalarsRequest {
	const RESPONSE_SIZE: usize = size_of::<u8>();

	type Response = ();

	fn serialize_request(&self) -> Bytes {
		let mut req = BytesMut::with_capacity(1 + 6);
		req.put_u8(b's');

		req.put_i8(self.linear.inner);
		req.put_i8(self.angular.inner);

		req.freeze()
	}

	fn parse_response(input: &[u8]) -> nom::IResult<&[u8], Self::Response> {
		nom::combinator::all_consuming(assert_byte_tag(b'd')).parse(input)
	}
}
