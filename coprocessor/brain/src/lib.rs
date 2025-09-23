#![no_std]

use crc::Crc;

const CRC_ALG: crc::Algorithm<u16> = crc::Algorithm {
    width: 16,
    poly: 0xa2eb,
    init: 0xffff,
    refin: false,
    refout: false,
    xorout: 0xffff,
    check: 0x624e,
    residue: 0xddfc,
};
const CRC: Crc<u16> = Crc::<u16>::new(&CRC_ALG);

pub struct Packet {
    id: u8,
    data: BrainPacketData,
}

pub enum BrainPacketData {
    CalibrateOtos,
}
