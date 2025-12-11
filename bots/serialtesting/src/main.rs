#![feature(never_type, future_join)]
#![allow(unused_variables, unused_imports)] // this is just testing, often code won't be used

use std::{
    io::{Read, Write},
    time::{Duration, Instant},
};

use coprocessor::{requests::SetLedRequest, vexide::CoprocessorSmartPort};
use push_back::subsystems::{copro::CoproSubsystem, intake::IntakeState};
use vex_sdk::{
    V5_AdiPortConfiguration, V5_DeviceT, vexDeviceAdiPortConfigSet, vexDeviceAdiValueGet,
    vexDeviceGetByIndex,
};
use vexide::{controller::ControllerState, prelude::*};

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let (copro, mut data) = CoprocessorSmartPort::new(peripherals.port_1).await;

    let mut i = 0u16;
    loop {
        if i.is_multiple_of(5) {
            println!("{:?}", data.position.read().heading);
        }


        i = i.wrapping_add(1);
        sleep(Duration::from_millis(10)).await;
    }
}
