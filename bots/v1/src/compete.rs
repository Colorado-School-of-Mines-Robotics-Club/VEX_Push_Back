use std::{collections::HashMap, fs::File, time::Duration};

use evian::prelude::Arcade;
use push_back::subsystems::{intake::IntakeState, ControllableSubsystem};
use vexide::prelude::*;

use crate::robot::Robot;

impl Compete for Robot {
    async fn driver(&mut self) {
        println!("Driver!");
        _ = self.controller.set_text(self.configuration.as_str(), 1, 1).await;

        let mut i: usize = 0;
        loop {
            if let Ok(controller) = self.controller.state() {
                if controller.button_up.is_now_pressed() {
                    self.configuration = self.configuration.next();
                    _ = self.controller.set_text(self.configuration.as_str(), 1, 1).await
                }

                // Run subsystems
                self.drivetrain.update(&controller, self.configuration);
                // self.coprocessor.control(&controller, self.configuration);
                self.intake.update(&controller, self.configuration);
                self.trunk.update(&controller, self.configuration);

                self.replay.record(&controller, &[
                    ("drivetrain", &self.drivetrain),
                    ("intake", &self.intake),
                    ("trunk", &self.trunk),
                ]);
            } else if i.is_multiple_of(100) {
                // Only print every second
                println!("Warning: controller not connected");
            }

            i = i.wrapping_add(1);
            sleep(Duration::from_millis(10)).await;
        }
    }

    async fn autonomous(&mut self) {
        println!("Auton!");

        if let Ok(file) = File::options()
            .read(true)
            .open("record30.txt") {
                let mut subsystems = HashMap::<&'static str, &mut dyn ControllableSubsystem>::new();
                subsystems.insert("drivetrain", &mut self.drivetrain);
                subsystems.insert("intake", &mut self.intake);
                subsystems.insert("trunk", &mut self.trunk);
                self.replay.replay(file, subsystems).await;
            }

        // _ = self.intake.run(IntakeState::full_forward());
        // sleep(Duration::from_secs(1)).await;
        // _ = self.intake.run(IntakeState::full_brake());

        // _ = self.drivetrain.drivetrain.model.drive_arcade(-0.5, 0.0);
        // sleep(Duration::from_millis(750)).await;
        // _ = self.drivetrain.drivetrain.model.drive_arcade(0.0, 0.0);
        // for _ in 0..5 {
        //     _ = self.intake.run(IntakeState::INTAKE_WHEELS);
        //     sleep(Duration::from_millis(250)).await;
        //     _ = self.intake.run(IntakeState::full_brake());
        //     sleep(Duration::from_millis(750)).await;
        // }
        // _ = self.drivetrain.drivetrain.model.drive_arcade(1.0, 0.0);
        // sleep(Duration::from_secs(10)).await;
        // _ = self.drivetrain.drivetrain.model.drive_arcade(0.0, 0.0);

        // crate::autons::print_pose(self).await;
        // crate::autons::auton_1(self).await;
        // crate::autons::tune_pid(self).await;
    }

    async fn disabled(&mut self) {
        println!("Disabled!")
    }
}
