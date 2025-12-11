use bytes::Bytes;
use nom::Parser as _;

use crate::requests::{CoprocessorRequest, assert_byte_tag};

pub struct SetLedRequest;

impl CoprocessorRequest for SetLedRequest {
    const RESPONSE_SIZE: usize = size_of::<u8>();

    type Response = ();

    fn serialize_request(&self) -> Bytes {
        Bytes::from_static(b"l")
    }

    fn parse_response(input: &[u8]) -> nom::IResult<&[u8], Self::Response> {
        nom::combinator::all_consuming(assert_byte_tag(b'd')).parse(input)
    }
}
