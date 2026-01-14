#![feature(never_type, future_join)]
#![allow(unused_variables, unused_imports)] // this is just testing, often code won't be used

use std::{
    fmt::Write as _, io::{Read, Write as _}, time::{Duration, Instant}
};

use coprocessor::{requests::{CalibrateRequest, CoprocessorRequest, GetPositionRequest, GetVelocityRequest, PingRequest, SetLedRequest}, vexide::CoprocessorSmartPort};
use evian::math::Vec2;
use push_back::subsystems::{copro::CoproSubsystem, intake::IntakeState};
use shrewnit::{Degrees, FeetPerSecond, Inches};
use vex_sdk::{
    V5_AdiPortConfiguration, V5_DeviceT, vexDeviceAdiPortConfigSet, vexDeviceAdiValueGet,
    vexDeviceGetByIndex,
};
use vexide::{color::Color, controller::ControllerState, prelude::*};

#[vexide::main]
async fn main(mut peripherals: Peripherals) {
    let (copro, mut data) = CoprocessorSmartPort::new(peripherals.port_15).await;

    sleep(Duration::from_secs(2)).await;

    let mut i = 0;
    let mut last_presses = 0;
    loop {
        let current_presses = peripherals.display.touch_status().press_count;
        i += 1;

        println!("Running tests attempt {i}:");

        run_tests(current_presses != last_presses, &copro, &mut peripherals.display).await;

        last_presses = current_presses;

        sleep(Duration::from_secs(2)).await;
        println!();
    }
}

async fn run_tests(calibration: bool, port: &CoprocessorSmartPort, display: &mut Display) {
    let mut results = vec![];

    if calibration {
        results.push(match port.send_request(CalibrateRequest).await {
            Ok(()) => "PASS - Calibration test".to_owned(),
            Err(e) => format!("FAIL - Calibration test: {e:?}"),
        });
    }

    // Test ping
    results.push(match port.send_request(PingRequest).await {
        Ok(hash) => format!("PASS - Ping test, up to date: {}", PingRequest::verify_hash(hash)),
        Err(e) => format!("FAIL - Ping test: {e:?}"),
    });

    // Test position
    results.push(match port.send_request(GetPositionRequest).await {
        Ok(pos) => format!("PASS - Position test, heading: {:.4}deg", pos.heading.to::<Degrees>()),
        Err(e) => format!("FAIL - Position test: {e:?}"),
    });

    // Test velocity
    results.push(match port.send_request(GetVelocityRequest).await {
        Ok(vel) => format!("PASS - Velocity test, speed: {:.4}in/s", (vel.x.to::<FeetPerSecond>().powi(2) + vel.y.to::<FeetPerSecond>().powi(2)).sqrt() * 12.0),
        Err(e) => format!("FAIL - Velocity test: {e:?}"),
    });

    for result in results {
        println!("{}", result);
        _ = write!(display, "{}\n", result);
    }
    _ = write!(display, "\n\n");
}
