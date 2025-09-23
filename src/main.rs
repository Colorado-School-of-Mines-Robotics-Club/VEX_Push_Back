#![no_main]
#![no_std]
#![feature(never_type)]

extern crate alloc;

mod compete;
// mod display;
mod robot;
mod tracking;

use core::time::Duration;

use vexide::{io::stdout, prelude::*};

use crate::robot::Robot;

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let robot = Robot::new(peripherals).await;

    robot.start().await;
}
