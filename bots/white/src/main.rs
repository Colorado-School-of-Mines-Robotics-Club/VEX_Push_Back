#![feature(never_type, future_join)]

mod robot;
mod compete;
mod control;

use vexide::prelude::*;

use crate::robot::Robot;

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let robot = Robot::new(peripherals).await;

    robot.start().await;
}
