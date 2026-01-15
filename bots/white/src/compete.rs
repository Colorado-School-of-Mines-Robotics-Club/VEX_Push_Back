use std::time::Duration;

use evian::math::Angle;
use evian::prelude::*;
use push_back::subsystems::intake::IntakeState;

use push_back::subsystems::ControllableSubsystem;
use push_back::subsystems::trunk::TrunkState;
use vexide::prelude::*;

use crate::robot::Robot;

async fn leo_auton_skills(robot: &mut Robot) {
    // Code from Leo, still a work in progress.

        // The robot turns left, drives forward, then turns right and drives towards the first ball.

    _ = robot.drivetrain.model.drive_tank(0.5, 0.5);
    sleep(Duration::from_secs(1)).await;
    _ = robot.drivetrain.model.drive_tank(0.5, -0.5);
    sleep(Duration::from_millis(450)).await;
    _ = robot.trunk.set_state(push_back::subsystems::trunk::TrunkState::Upper);
    _ = robot.drivetrain.model.drive_tank(0.5, 0.5);
    sleep(Duration::from_millis(600)).await;
    _ = robot.drivetrain.model.drive_tank(0.0, 0.0);

        // Robot begans intaking the balls from the pillar.

    _ = robot.intake.run(IntakeState::FULL);
    sleep(Duration::from_secs(2)).await;
    _ = robot.intake.run(IntakeState::empty());


        // Robot positions itself in front of the beams and ejects its balls.

    _ = robot.drivetrain.model.drive_tank(-0.5, -0.5);
    sleep(Duration::from_millis(500)).await;
    _ = robot.trunk.set_state(push_back::subsystems::trunk::TrunkState::Lower);
    _ = robot.drivetrain.model.drive_tank(-0.5, 0.5);
    sleep(Duration::from_millis(956)).await;                // This line of code only works if the robot is placed in an exact spot. Someone please fix this when we get the position sensor installed on the bot.
    _ = robot.drivetrain.model.drive_tank(0.0, 0.0);
    _ = robot.drivetrain.model.drive_tank(0.5, 0.5);
    sleep(Duration::from_millis(450)).await;
    _ = robot.drivetrain.model.drive_tank(0.0, 0.0);

    _ = robot.intake.run(IntakeState::FULL);

    sleep(Duration::from_secs(3)).await;

    //Remove after making final phase work.

        // Robot goes to the nearest balls and picks them up.

    _ = robot.trunk.set_state(push_back::subsystems::trunk::TrunkState::Down);

    _ = robot.drivetrain.model.drive_tank(0.5, -0.5);
    sleep(Duration::from_millis(600)).await;
    _ = robot.drivetrain.model.drive_tank(0.5, 0.5);
    sleep(Duration::from_millis(450)).await;
    _ = robot.drivetrain.model.drive_tank(-0.5, 0.5);
    sleep(Duration::from_millis(450)).await;
    _ = robot.drivetrain.model.drive_tank(0.5, 0.9);
    sleep(Duration::from_millis(1200)).await;
    _ = robot.drivetrain.model.drive_tank(0.0, 0.0);

    _ = robot.intake.run(IntakeState::BOTTOM);
    _ = robot.intake.run(IntakeState::ELEVATOR);
}

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
                self.coprocessor.control(&controller, self.configuration);
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
        // leo_auton_skills(self).await;

        let mut basic = crate::control::basic::CONTROLLER;

        // let start = self.drivetrain.tracking.forward_travel();
        // println!("START: {:.2}", start);

        // basic.drive_distance(&mut self.drivetrain, 12.0).await;

        // let end = self.drivetrain.tracking.forward_travel();
        // println!("END: {:.2}\nDIFF: {:.2}", end, end - start);

        let start = self.drivetrain.tracking.heading();
        println!("START: {:.2}", start.as_degrees());
        basic.turn_to_heading(&mut self.drivetrain, (start + Angle::from_degrees(90.0)).wrapped_full()).await;
        let end = self.drivetrain.tracking.heading();
        println!("END: {:.2}\nDIFF: {:.2}", end.as_degrees(), (end - start).wrapped_full().as_degrees());
    }

    async fn disabled(&mut self) {
        println!("Disabled!")
    }
}
