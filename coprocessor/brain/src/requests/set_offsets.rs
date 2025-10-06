use bytes::{BufMut, Bytes, BytesMut};
use nom::Parser as _;
use shrewnit::{Angle, Length};

use crate::{
    OtosAngle, OtosLength,
    requests::{CoprocessorRequest, assert_byte_tag},
};

pub struct SetOffsetsRequest {
    pub x: Length<f64>,
    pub y: Length<f64>,
    pub heading: Angle<f64>,
}

impl CoprocessorRequest for SetOffsetsRequest {
    const RESPONSE_SIZE: usize = size_of::<u8>();
    type Response = ();

    fn serialize_request(&self) -> Bytes {
        let mut req = BytesMut::with_capacity(1 + 6);
        req.put_u8(b'o');

        req.put_i16_le(self.x.to::<OtosLength>() as i16);
        req.put_i16_le(self.y.to::<OtosLength>() as i16);
        req.put_i16_le(self.heading.to::<OtosAngle>() as i16);

        req.freeze()
    }

    fn parse_response(input: &[u8]) -> nom::IResult<&[u8], Self::Response> {
        nom::combinator::all_consuming(assert_byte_tag(b'd')).parse(input)
    }
}
