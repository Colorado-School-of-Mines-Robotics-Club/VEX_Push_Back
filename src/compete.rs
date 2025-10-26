
use core::time::Duration;

use coprocessor::requests::CalibrateRequest;
// use autons::prelude::SelectCompete;
use evian::{math::Angle, motion::{basic::{DriveDistanceAtHeadingFuture, TurnToPointFuture}, Basic}, prelude::{Arcade, TracksHeading}};
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

        dbg!(self.drivetrain.tracking.heading());

        // The following coded is intended to make the robot drive forward, turn left, drive forward again, 
        // grab a triball, drive backwards, turn left and drive forward.

        basic.drive_distance(&mut self.drivetrain,10.0).await;
        basic.turn_to_heading(&mut self.drivetrain, Angle::from_degrees(90.0)).await;
        basic.drive_distance(&mut self.drivetrain,5.0).await;
        self.intake.set_velocity(200);
        sleep(Duration::from_secs(1));
        self.intake.set_velocity(0);
        basic.drive_distance(&mut self.drivetrain,5.0).await;
        basic.turn_to_heading(&mut self.drivetrain, Angle::from_degrees(180.0)).await;
            
        

    }

    async fn disabled(&mut self) {
        println!("Disabled!")
    }
}
