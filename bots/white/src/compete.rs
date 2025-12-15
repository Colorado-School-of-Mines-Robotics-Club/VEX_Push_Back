use std::time::Duration;

use evian::prelude::Tank;
use push_back::subsystems::intake::IntakeState;

use push_back::subsystems::ControllableSubsystem;
use push_back::subsystems::trunk::TrunkState;
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

        // Code from Leo, still a work in progress.

            // The robot turns left, drives forward, then turns right and drives towards the first ball.
        
        self.drivetrain.model.drive_tank(0.5, 0.5);
        sleep(Duration::from_secs(1)).await;
        self.drivetrain.model.drive_tank(0.5, -0.5);
        sleep(Duration::from_millis(450)).await;
        self.trunk.set_state(push_back::subsystems::trunk::TrunkState::Upper);
        self.drivetrain.model.drive_tank(0.5, 0.5);
        sleep(Duration::from_millis(600)).await;
        self.drivetrain.model.drive_tank(0.0, 0.0);

            // Robot begans intaking the balls from the pillar.

        self.intake.run(IntakeState::FULL);
        sleep(Duration::from_secs(2)).await;
        self.intake.run(IntakeState::empty());


            // Robot positions itself in front of the beams and ejects its balls.

        self.drivetrain.model.drive_tank(-0.5, -0.5);
        sleep(Duration::from_millis(500)).await;
        self.trunk.set_state(push_back::subsystems::trunk::TrunkState::Lower);
        self.drivetrain.model.drive_tank(-0.5, 0.5);
        sleep(Duration::from_millis(956)).await;                // This line of code only works if the robot is placed in an exact spot. Someone please fix this when we get the position sensor installed on the bot.
        self.drivetrain.model.drive_tank(0.0, 0.0);
        self.drivetrain.model.drive_tank(0.5, 0.5);
        sleep(Duration::from_millis(450)).await;
        self.drivetrain.model.drive_tank(0.0, 0.0);

        self.intake.run(IntakeState::FULL);

        sleep(Duration::from_secs(3)).await;

        //Remove after making final phase work.

            // Robot goes to the nearest balls and picks them up.

        self.trunk.set_state(push_back::subsystems::trunk::TrunkState::Down);

        self.drivetrain.model.drive_tank(0.5, -0.5);
        sleep(Duration::from_millis(600)).await;
        self.drivetrain.model.drive_tank(0.5, 0.5);
        sleep(Duration::from_millis(450)).await;
        self.drivetrain.model.drive_tank(-0.5, 0.5);
        sleep(Duration::from_millis(450)).await;
        self.drivetrain.model.drive_tank(0.5, 0.9);
        sleep(Duration::from_millis(1200)).await;
        self.drivetrain.model.drive_tank(0.0, 0.0);
        
        self.intake.run(IntakeState::BOTTOM);
        self.intake.run(IntakeState::ELEVATOR);

    }
     // End of Leo's Code.

    async fn disabled(&mut self) {
        println!("Disabled!")
    }
}
