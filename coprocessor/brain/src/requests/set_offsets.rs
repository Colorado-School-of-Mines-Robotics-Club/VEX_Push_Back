use bytes::{BufMut, Bytes, BytesMut};
use nom::Parser as _;
use shrewnit::{Angle, Length};

use crate::{
    OtosAngle, OtosLength,
    requests::{CoprocessorRequest, OtosPosition, assert_byte_tag},
};

pub struct SetOffsetsRequest(pub OtosPosition);

impl CoprocessorRequest for SetOffsetsRequest {
    const RESPONSE_SIZE: usize = size_of::<u8>();

    type Response = ();

    fn serialize_request(&self) -> Bytes {
        let mut req = BytesMut::with_capacity(1 + 6);
        req.put_u8(b'o');

        req.put_i16_le(self.0.x.to::<OtosLength>() as i16);
        req.put_i16_le(self.0.y.to::<OtosLength>() as i16);
        req.put_i16_le(dbg!(self.0.heading.to::<OtosAngle>()) as i16);

        dbg!(req.freeze())
    }

    fn parse_response(input: &[u8]) -> nom::IResult<&[u8], Self::Response> {
        nom::combinator::all_consuming(assert_byte_tag(b'd')).parse(input)
    }
}
