#![feature(never_type, future_join, duration_millis_float)]

mod robot;
mod compete;

use vexide::prelude::*;

use crate::robot::Robot;

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let robot = Robot::new(peripherals).await;

    robot.start().await;
}
