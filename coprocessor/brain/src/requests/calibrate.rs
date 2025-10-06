use core::time::Duration;

use bytes::Bytes;
use nom::Parser as _;

use crate::requests::{CoprocessorRequest, assert_byte_tag};

pub struct CalibrateRequest;

impl CoprocessorRequest for CalibrateRequest {
    const RESPONSE_SIZE: usize = size_of::<u8>();
    const TIMEOUT: Duration = Duration::from_millis(1500);

    type Response = ();

    fn serialize_request(&self) -> Bytes {
        Bytes::from_static(b"c")
    }

    fn parse_response(input: &[u8]) -> nom::IResult<&[u8], Self::Response> {
        nom::combinator::all_consuming(assert_byte_tag(b'd')).parse(input)
    }
}
