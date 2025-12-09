#![feature(never_type, future_join)]
#![allow(unused_variables, unused_imports)] // this is just testing, often code won't be used

use std::{
    io::{Read, Write},
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
async fn main(peripherals: Peripherals) {
    let mut port = SerialPort::open(peripherals.port_1, 921600).await;
    let mut buf = [0u8; 2048];
    let start = Instant::now();
    loop {
        let mut out = std::io::stdout().lock();
        while let Ok(len) = port.read(&mut buf)
            && len > 0
        {
            _ = out.write_all(&buf[0..len]);
        }

        if let Ok(controller) = peripherals.primary_controller.state()
            && controller.button_a.is_now_pressed()
        {
            _ = port.write_all(b"a ");
            _ = port.write_all(start.elapsed().as_secs().to_string().as_bytes());
        }
        sleep(Duration::from_millis(10)).await;
    }
}
