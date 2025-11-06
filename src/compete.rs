use core::time::Duration;

use coprocessor::{requests::CalibrateRequest, vexide::CoprocessorSmartPort};
use evian::{
    math::Angle,
    motion::Basic,
    prelude::{Arcade, Tank, TracksHeading, TracksPosition},
};
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
                // .drive_tank(state.left_stick.y(), state.right_stick.y());
                .drive_arcade(state.left_stick.y(), state.left_stick.x() * 0.5);

            // Intake control
            // let _ = self.intake.set_voltage(
            //     match (state.button_r1.is_pressed(), state.button_r2.is_pressed()) {
            //         (false, false) | (true, true) => 0_f64,
            //         (true, false) => self.intake.max_voltage(),
            //         (false, true) => -self.intake.max_voltage(),
            //     },
            // );

            // Calibrate OTOS button
            if state.button_x.is_now_pressed() {
                let request_future = self.coprocessor.send_request(CalibrateRequest);
                vexide::task::spawn(async move {
                    println!("Starting calibration...");
                    match request_future.await {
                        Ok(_) => println!("Calibrated!"),
                        Err(e) => println!("Unable to calibrate: {e:?}"),
                    }
                })
                .detach();
            }

            sleep(Duration::from_millis(10)).await;
        }
    }

    async fn autonomous(&mut self) {
        println!("Auton!");

        crate::autons::print_pose(self).await;
        // crate::autons::auton_1(self).await;
        // crate::autons::tune_pid(self).await;
    }

    async fn disabled(&mut self) {
        println!("Disabled!")
    }
}
