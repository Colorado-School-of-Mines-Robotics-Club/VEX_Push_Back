#![feature(never_type, future_join)]
#![allow(unused_variables, unused_imports)] // this is just testing, often code won't be used

use std::{
    io::{Read, Write},
    time::{Duration, Instant},
};

use ciborium::value::Integer;
use push_back::subsystems::{intake::IntakeState, replay::structs::RecordingEntry};
use vex_sdk::{
    V5_AdiPortConfiguration, V5_DeviceT, vexDeviceAdiPortConfigSet, vexDeviceAdiValueGet,
    vexDeviceGetByIndex,
};
use vexide::{controller::ControllerState, prelude::*};

const FILE: &str = "record120.txt";

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let mut file = std::fs::read(FILE).unwrap();
    let mut slice = file.as_mut_slice();
    slice = slice
        .split_at_mut(slice.iter().position(|b| *b == b'\n').unwrap() + 1)
        .1;
    while let Some(null_pos) = slice.iter().position(|b| *b == b'\0') {
        let (msg, rest) = slice.split_at_mut(null_pos + 1);
        slice = rest;

        let len = cobs::decode_in_place(msg).unwrap();
        let parsed = ciborium::from_reader::<RecordingEntry, _>(&msg[0..len]).unwrap();
        let intake_state = parsed
            .subsystem_states
            .iter()
            .find(|(name, _)| name == "intake")
            .unwrap()
            .1
            .as_integer()
            .unwrap();
        dbg!(intake_state);
    }
}
