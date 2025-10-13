use bytes::Bytes;
use nom::Parser as _;
use shrewnit::{Angle, Length};

use crate::requests::{CoprocessorRequest, parse_i16_triple};

#[derive(Debug, PartialEq, Default, Clone, Copy)]
pub struct OtosPosition {
    /// The absolute positioning in the x direction
    pub x: Length<f64>,
    /// The absolute positioning in the y direction
    pub y: Length<f64>,
    /// The current heading of the robot, with an atan2-style [-180,180] range
    pub heading: Angle<f64>,
}

unsafe impl bytemuck::NoUninit for OtosPosition {}

pub struct GetPositionRequest;

impl CoprocessorRequest for GetPositionRequest {
    const RESPONSE_SIZE: usize = size_of::<i16>() * 3;

    type Response = OtosPosition;

    fn serialize_request(&self) -> Bytes {
        Bytes::from_static(b"p")
    }

    fn parse_response(input: &[u8]) -> nom::IResult<&[u8], Self::Response> {
        nom::combinator::map(
            nom::combinator::all_consuming(parse_i16_triple),
            |(x, y, h)| OtosPosition {
                x: (x as f64) * crate::OtosLength,
                y: (y as f64) * crate::OtosLength,
                heading: (h as f64) * crate::OtosAngle,
            },
        )
        .parse(input)
    }
}
