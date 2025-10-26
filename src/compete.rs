use core::time::Duration;

use coprocessor::requests::CalibrateRequest;
// use autons::prelude::SelectCompete;
use evian::{math::Angle, motion::Basic, prelude::{Arcade, TracksHeading}};
use shrewnit::{Degrees, Radians};
use vexide::prelude::*;

use crate::robot::Robot;

impl Compete for Robot {
    async fn driver(&mut self) {
        println!("Driver!");

        loop {
            let Ok(state) = self.controller.state() else {
                // println!("Warning: controller not connected");
                sleep(Duration::from_millis(10)).await;
                continue;
            };

            // Tank drive
            let _ = self
                .drivetrain
                .model
                .drive_arcade(state.left_stick.y(), state.left_stick.x() * 0.5);

            // Intake control
            let _ = self.intake.set_voltage(
                match (state.button_r1.is_pressed(), state.button_r2.is_pressed()) {
                    (false, false) | (true, true) => 0_f64,
                    (true, false) => self.intake.max_voltage(),
                    (false, true) => -self.intake.max_voltage(),
                },
            );

            // Calibrate OTOS button
            if state.button_x.is_now_pressed() {
                match self.coprocessor.send_request(CalibrateRequest).await {
                    Ok(_) => println!("Calibrated!"),
                    Err(e) => println!("Unable to calibrate: {e:?}"),
                };
            }

            sleep(Duration::from_millis(10)).await;
        }
    }

    async fn autonomous(&mut self) {
        println!("Auton!");

        let mut basic = Basic {
            linear_controller: Robot::LINEAR_PID,
            angular_controller: Robot::ANGULAR_PID,
            linear_tolerances: Robot::LINEAR_TOLERANCES,
            angular_tolerances: Robot::ANGULAR_TOLERANCES,
            timeout: None,
        };

        // basic.drive_distance(&mut self.drivetrain, -12.0).await;

        // loop {
        //     let current_angle = self.drivetrain.tracking.heading();

        //     dbg!(current_angle.as_degrees());
        //     sleep(Duration::from_millis(500)).await
        // }

        dbg!(self.drivetrain.tracking.heading());

        basic.turn_to_heading(
            &mut self.drivetrain,
            Angle::from_radians(0.0)
        ).await;
    }

    async fn disabled(&mut self) {
        println!("Disabled!")
    }
}
