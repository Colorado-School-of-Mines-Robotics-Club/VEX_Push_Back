use std::time::Duration;

use push_back::subsystems::ControllableSubsystem as _;
use vexide::prelude::*;

use crate::robot::Robot;

impl Compete for Robot {
    async fn driver(&mut self) {
        println!("Driver!");

        let mut i: usize = 0;
        loop {
            if let Ok(controller) = self.controller.state() {
                if controller.button_right.is_now_pressed() {
                    self.configuration = self.configuration.next();
                    _ = self.controller.screen.set_text(self.configuration.as_str(), 1, 1).await
                }

                // Run subsystems
                // self.replay.control(&mut controller);
                self.drivetrain.update(&controller, self.configuration);
                // self.coprocessor.control(&controller, self.configuration);
                self.intake.update(&controller, self.configuration);
                self.trunk.update(&controller, self.configuration);
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

        // self.replay.

        // crate::autons::print_pose(self).await;
        // crate::autons::auton_1(self).await;
        // crate::autons::tune_pid(self).await;
    }

    async fn disabled(&mut self) {
        println!("Disabled!")
    }
}
