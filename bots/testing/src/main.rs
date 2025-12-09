#![feature(never_type, future_join)]
#![allow(unused_variables, unused_imports)] // this is just testing, often code won't be used

use std::{
    fs::File,
    io::Write,
    time::{Duration, Instant},
};

use push_back::subsystems::intake::IntakeState;
use vex_sdk::{
    V5_AdiPortConfiguration, V5_DeviceT, vexDeviceAdiPortConfigSet, vexDeviceAdiValueGet,
    vexDeviceGetByIndex,
};
use vexide::{controller::ControllerState, prelude::*};

const FILE: &str = "testing.txt";

#[vexide::main]
async fn main(mut peripherals: Peripherals) {
    // println!("{}", IntakeState::INTAKE_WHEELS.bits());
    // println!("{}", IntakeState::INTAKE_WHEELS.reverse().bits());
    // match std::fs::read(FILE) {
    //     Ok(contents) => println!(
    //         "File contents:\n{}\nEOF",
    //         contents
    //             .utf8_chunks()
    //             .map(|e| e.valid().to_string() + &"�".repeat(e.invalid().len()))
    //             .collect::<Vec<String>>()
    //             .join("?")
    //     ),
    //     Err(e) => println!("Error opening file: {e:?}"),
    // }
    // let adi_expander = unsafe { vexDeviceGetByIndex(22 - 1) };
    // unsafe { vexDeviceAdiPortConfigSet(adi_expander, 0, V5_AdiPortConfiguration::kAdiPortTypeDigitalIn) };
    // let port = AdiDigitalIn::new(peripherals.adi_a);
    // loop {
    //     // dbg!(unsafe { vexDeviceAdiValueGet(adi_expander, 0) });
    //     dbg!(port.level());
    //     sleep(Duration::from_millis(100)).await;
    // }
    sleep(Duration::from_secs(1)).await;
    // for _ in 0..3 {
    //     peripherals.primary_controller.rumble("-").await.unwrap();
    //     sleep(Duration::from_secs(1)).await;
    // }
    // peripherals.primary_controller.rumble("..").await.unwrap();
    // dbg!(1);
    peripherals
        .primary_controller
        .set_text("123456789abcdefghijklmnopqrstuvwxyz", 1, 1)
        .await
        .unwrap();
    sleep(Duration::from_secs(1000)).await;
}
