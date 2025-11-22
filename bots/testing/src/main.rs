#![feature(never_type, future_join)]

use std::{
    fs::File,
    io::Write,
    time::{Duration, Instant},
};

use push_back::subsystems::intake::IntakeState;
use vexide::{controller::ControllerState, prelude::*};

const FILE: &str = "testing.txt";

#[vexide::main]
async fn main(peripherals: Peripherals) {
    // println!("{}", IntakeState::INTAKE_WHEELS.bits());
    // println!("{}", IntakeState::INTAKE_WHEELS.reverse().bits());
    match std::fs::read(FILE) {
        Ok(contents) => println!(
            "File contents:\n{}\nEOF",
            contents
                .utf8_chunks()
                .map(|e| e.valid().to_string() + &"�".repeat(e.invalid().len()))
                .collect::<Vec<String>>()
                .join("?")
        ),
        Err(e) => println!("Error opening file: {e:?}"),
    }
    std::process::exit(0);
}
