use std::time::Duration;

use push_back::subsystems::ControllableSubsystem;
use vexide::prelude::*;

use crate::robot::Robot;

impl Compete for Robot {
    async fn driver(&mut self) {
        println!("Driver!");
        _ = self
            .controller
            .set_text(self.configuration.as_str(), 1, 1)
            .await;

        let mut i: usize = 0;
        loop {
            if let Ok(controller) = self.controller.state() {
                if controller.button_up.is_now_pressed() {
                    self.configuration = self.configuration.next();
                    _ = self
                        .controller
                        .set_text(self.configuration.as_str(), 1, 1)
                        .await
                }

                // Run subsystems
                self.drivetrain.control(&controller, self.configuration);
                // self.coprocessor.control(&controller, self.configuration);
                self.intake.control(&controller, self.configuration);
                self.trunk.control(&controller, self.configuration);

                self.replay.record(
                    &controller,
                    &[
                        ("drivetrain", &self.drivetrain),
                        ("intake", &self.intake),
                        ("trunk", &self.trunk),
                    ],
                );
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

        // if let Ok(file) = File::options().read(true).open("record30.txt") {
        //     let mut subsystems = HashMap::<&'static str, &mut dyn ControllableSubsystem>::new();
        //     subsystems.insert("drivetrain", &mut self.drivetrain);
        //     subsystems.insert("intake", &mut self.intake);
        //     subsystems.insert("trunk", &mut self.trunk);
        //     self.replay.replay(file, subsystems).await;
        // }

        // crate::autons::print_pose(self).await;
        // crate::autons::auton_1(self).await;
        // crate::autons::tune_pid(self).await;
        // crate::autons::print_state(self).await;
        // crate::autons::throw_balls(self).await;
    }

    async fn disabled(&mut self) {
        println!("Disabled!")
    }
}
