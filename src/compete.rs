use core::time::Duration;

// use autons::prelude::SelectCompete;
use evian::prelude::{Arcade, Tank};
use vexide::prelude::*;

use crate::robot::Robot;

impl Compete for Robot {
    async fn driver(&mut self) {
        println!("Driver!");

        loop {
            let Ok(state) = self.controller.state() else {
                println!("Warning: controller not connected");
                continue;
            };

            // Tank drive
            let _ = self
                .drivetrain
                .model
                .drive_arcade(state.left_stick.y(), state.left_stick.x());
                // .drive_tank(state.left_stick.y(), state.right_stick.y());

            // Intake control
            let _ = self.intake.set_voltage(
                match (state.button_r1.is_pressed(), state.button_r2.is_pressed()) {
                    (false, false) | (true, true) => 0_f64,
                    (true, false) => self.intake.max_voltage(),
                    (false, true) => -self.intake.max_voltage(),
                },
            );

            sleep(Duration::from_millis(10)).await;
        }
    }

    async fn autonomous(&mut self) {
        println!("Auton!")
    }

    async fn disabled(&mut self) {
        println!("Disabled!")
    }
}
