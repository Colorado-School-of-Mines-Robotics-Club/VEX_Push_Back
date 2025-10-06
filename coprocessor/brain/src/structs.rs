use alloc::vec::Vec;
use bytes::{BufMut as _, BytesMut};

use crate::{OtosAngle, OtosLength};

#[repr(u8)]
#[derive(Debug, PartialEq)]
pub enum CoproRequest {
    Calibrate,
    SetOtosOffset(crate::otos::OtosPosition),
    GetOtosPosition,
    GetOtosVelocity,
}

impl CoproRequest {
    /// Serializes the [CoproRequest] into COBS-encoded raw bytes to be sent over serial
    pub fn serialize(&self) -> Vec<u8> {
        let mut data = BytesMut::with_capacity(1);

        match self {
            CoproRequest::Calibrate => {
                data.put_u8(b'c');
            }
            CoproRequest::SetOtosOffset(offset) => {
                data.put_u8(b'c');

                data.reserve(3 * size_of::<i16>());
                data.put_i16_le(offset.x.to::<OtosLength>() as i16);
                data.put_i16_le(offset.y.to::<OtosLength>() as i16);
                data.put_i16_le(offset.heading.to::<OtosAngle>() as i16);
            }
            CoproRequest::GetOtosPosition => {
                data.put_u8(b'p');
            }
            CoproRequest::GetOtosVelocity => {
                data.put_u8(b'v');
            }
        };

        cobs::encode_vec(&data)
    }
}
