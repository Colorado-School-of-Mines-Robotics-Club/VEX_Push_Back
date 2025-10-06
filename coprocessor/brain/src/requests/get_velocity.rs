use bytes::Bytes;
use nom::Parser as _;
use shrewnit::{AngularVelocity, LinearVelocity};

use crate::requests::{CoprocessorRequest, parse_i16_triple};

#[derive(Debug, PartialEq, Default)]
pub struct OtosVelocity {
    pub x: LinearVelocity<f64>,
    pub y: LinearVelocity<f64>,
    pub heading: AngularVelocity<f64>,
}

pub struct GetVelocityRequest;

impl CoprocessorRequest for GetVelocityRequest {
    const RESPONSE_SIZE: usize = size_of::<i16>() * 3;

    type Response = OtosVelocity;

    fn serialize_request(&self) -> Bytes {
        Bytes::from_static(b"v")
    }

    fn parse_response(input: &[u8]) -> nom::IResult<&[u8], Self::Response> {
        nom::combinator::map(
            nom::combinator::all_consuming(parse_i16_triple),
            |(x, y, h)| OtosVelocity {
                x: (x as f64) * crate::OtosLinearVelocity,
                y: (y as f64) * crate::OtosLinearVelocity,
                heading: (h as f64) * crate::OtosAngularVelocity,
            },
        )
        .parse(input)
    }
}
