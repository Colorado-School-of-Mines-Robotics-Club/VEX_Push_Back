#![feature(never_type, future_join)]

// mod autons;
mod compete;
mod robot;

use vexide::prelude::*;

use crate::robot::Robot;

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let robot = Robot::new(peripherals).await;

    robot.start().await;
}
