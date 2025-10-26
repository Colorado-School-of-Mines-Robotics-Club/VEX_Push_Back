#![no_main]
#![no_std]
#![feature(never_type, future_join)]

extern crate alloc;

mod autons;
mod compete;
mod robot;
mod tracking;

use vexide::prelude::*;

use crate::robot::Robot;

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let robot = Robot::new(peripherals).await;

    robot.start().await;
}
