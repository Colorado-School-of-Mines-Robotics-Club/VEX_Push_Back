#![feature(never_type, future_join)]

use v1::robot::{Robot, V1Bot};
use vexide::prelude::*;

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let robot = Robot::new(peripherals, V1Bot::Black).await;

    robot.start().await;
}
