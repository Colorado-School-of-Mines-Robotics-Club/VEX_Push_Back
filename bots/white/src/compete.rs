use std::time::{Duration, Instant};

use coprocessor::requests::CalibrateRequest;
use evian::math::{Angle, Vec2};
use evian::prelude::*;
use push_back::subsystems::intake::IntakeState;

use push_back::subsystems::trunk::TrunkState;
use push_back::subsystems::ControllableSubsystem;
use shrewnit::{FeetPerSecond, LinearVelocity, MetersPerSecond, Milliseconds};
use vexide::prelude::*;

use crate::robot::Robot;

async fn noah_auton_skills(robot: &mut Robot) {
    _ = robot.drivetrain.drivetrain.model.drive_tank(0.0, 0.0);
    println!("Auton!");
    let mut basic = crate::control::basic_noah::CONTROLLER;
    basic.turn_to_heading(&mut robot.drivetrain, (Angle::from_degrees(90.0-40.0)).wrapped_full()).await;
    basic.timeout = Some(Duration::from_secs(3));
    basic.drive_distance(&mut robot.drivetrain, 42.0).await;
    basic.timeout = Some(Duration::from_secs(1));
    basic.turn_to_heading(&mut robot.drivetrain, (Angle::from_degrees(90.0-70.0)).wrapped_full()).await;
    _ = robot.intake.run(IntakeState::full_forward() - IntakeState::TRUNK - IntakeState::ELEVATOR);
    sleep(Duration::from_millis(5)).await;
    basic.drive_distance(&mut robot.drivetrain, 7.0).await;
    _ = robot.intake.run(IntakeState::full_brake());
    basic.drive_distance(&mut robot.drivetrain, -7.0).await;
    basic.turn_to_heading(&mut robot.drivetrain, (Angle::from_degrees(180.0)).wrapped_full()).await;
    basic.timeout = Some(Duration::from_secs(3));
    basic.drive_distance(&mut robot.drivetrain, 23.0).await;
    basic.timeout = Some(Duration::from_secs(1));
    basic.turn_to_heading(&mut robot.drivetrain, (Angle::from_degrees(90.0 + 45.0)).wrapped_full()).await;
    basic.timeout = Some(Duration::from_millis(200));
    basic.drive_distance(&mut robot.drivetrain, 500.0).await;
    _ = robot.intake.run(IntakeState::full_forward());
    sleep(Duration::from_secs(3)).await;
    _ = robot.intake.run(IntakeState::full_brake());
    basic.timeout = Some(Duration::from_secs(1));
    basic.turn_to_heading(&mut robot.drivetrain, (Angle::from_degrees(90.0 + 45.0 - 180.0)).wrapped_full()).await;
    basic.timeout = Some(Duration::from_secs(3));
    _ = robot.trunk.set_state(TrunkState::Upper);
    basic.drive_distance_at_heading(&mut robot.drivetrain, 45.0, (Angle::from_degrees(90.0 + 45.0 + 170.0)).wrapped_full()).await;
    basic.timeout = Some(Duration::from_secs(1));
    basic.turn_to_heading(&mut robot.drivetrain, (Angle::from_degrees(-85.0 + 180.0)).wrapped_full()).await;
    basic.timeout = Some(Duration::from_millis(450));
    _ = robot.intake.run(IntakeState::full_forward() - IntakeState::TRUNK);
    basic.drive_distance(&mut robot.drivetrain, 500.0).await;
    sleep(Duration::from_secs(3)).await;
    _ = robot.intake.run(IntakeState::full_brake());
    basic.timeout = Some(Duration::from_secs(1));
    basic.drive_distance(&mut robot.drivetrain, -10.0).await;
    _ = robot.trunk.set_state(TrunkState::Lower);
    _ = robot.intake.run(IntakeState::full_reverse());
    sleep(Duration::from_millis(200)).await;
    _ = robot.intake.run(IntakeState::full_brake());
    basic.turn_to_heading(&mut robot.drivetrain, (Angle::from_degrees(85.5 + 180.0)).wrapped_full()).await;
    basic.drive_distance(&mut robot.drivetrain, 30.0).await;
    _ = robot.intake.run(IntakeState::full_forward());
    sleep(Duration::from_secs(3)).await;
    _ = robot.intake.run(IntakeState::full_reverse());
    sleep(Duration::from_millis(150)).await;
    _ = robot.intake.run(IntakeState::full_forward());
    sleep(Duration::from_secs(3)).await;
    _ = robot.intake.run(IntakeState::full_reverse());
    sleep(Duration::from_millis(150)).await;
    _ = robot.intake.run(IntakeState::full_forward());
    sleep(Duration::from_secs(3)).await;
    _ = robot.intake.run(IntakeState::full_reverse());
    sleep(Duration::from_millis(150)).await;
    _ = robot.intake.run(IntakeState::full_forward());
    sleep(Duration::from_secs(3)).await;
    _ = robot.intake.run(IntakeState::full_reverse());
    sleep(Duration::from_millis(150)).await;
}

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

        // push_back::autons::print_pose(&self.drivetrain.tracking).await;
        // crate::autons::auton_1(self).await;
        // crate::autons::tune_pid(self).await;
        // crate::autons::print_state(self).await;
        // crate::autons::throw_balls(self).await;
        // leo_auton_skills(self).await; return;
        // noah_auton_skills(self).await; return;

        let mut basic = crate::control::basic::CONTROLLER;

        // while self.coprocessor.send_request(CalibrateRequest).await.is_err() {
        //     eprintln!("Failed calibration...");
        //     sleep(Duration::from_secs(1)).await;
        // }
        // sleep(Duration::from_secs(1)).await;

        // let start = self.drivetrain.tracking.heading();
        // println!("START: {:.2}", start.as_degrees());
        // basic.turn_to_heading(&mut self.drivetrain, (start + Angle::from_degrees(90.0)).wrapped_full()).await;
        // let end = self.drivetrain.tracking.heading();
        // println!("END: {:.2}\nDIFF: {:.2}", end.as_degrees(), (end - start).wrapped_full().as_degrees());
        // return;

        // Move to correct balls on right
        basic.timeout = Some(Duration::from_secs(1));
        basic.turn_to_point(&mut self.drivetrain, Vec2::new(25.0, 24.0)).await;
        // basic.turn_to_point(&mut self.drivetrain, Vec2::new(22.0, 24.0)).await;
        basic.timeout = Some(Duration::from_secs(3));
        basic.drive_distance(&mut self.drivetrain, 28.0).await;
        // basic.drive_distance(&mut self.drivetrain, 32.0).await;

        basic.timeout = Some(Duration::from_secs(1));

        // Intake 2 team balls
        _ = self.intake.run(IntakeState::full_forward() - IntakeState::TRUNK - IntakeState::ELEVATOR);
        // sleep(Duration::from_millis(500)).await;
        basic.drive_distance(&mut self.drivetrain, 9.0).await;
        basic.drive_distance_at_heading(&mut self.drivetrain, 3.5, Angle::from_degrees(75.0)).await;
        sleep(Duration::from_secs(3)).await;
        _ = self.intake.run(IntakeState::full_brake());

        // Move to center low goal
        basic.drive_distance_at_heading(&mut self.drivetrain, -6.0, Angle::from_degrees(45.0)).await; // Avoid line

        let goal_load_pos = Vec2::new(0.0, 32.0);
        // let goal_load_pos = Vec2::new(0.0, 36.0);
        basic.turn_to_point(&mut self.drivetrain, goal_load_pos).await;
        basic.timeout = Some(Duration::from_secs(2));
        let dist = (self.drivetrain.tracking.position() - goal_load_pos).length();
        basic.drive_distance(&mut self.drivetrain, dist).await;

        basic.turn_to_heading(&mut self.drivetrain, Angle::from_degrees(135.0)).await; // Turn towards goal

        _ = self.drivetrain.model.drive_arcade(0.5, 0.0);
        sleep(Duration::from_millis(500)).await;
        _ = self.drivetrain.model.drive_arcade(0.0, 0.0);

        // Dump all 3 team balls into goal
        _ = self.trunk.set_state(TrunkState::Lower);
        _ = self.intake.run(IntakeState::full_reverse());
        sleep(Duration::from_secs(3)).await;
        _ = self.intake.run(IntakeState::full_forward());
        sleep(Duration::from_millis(500)).await;
        _ = self.intake.run(IntakeState::full_reverse());
        sleep(Duration::from_millis(1500)).await;
        _ = self.intake.run(IntakeState::full_forward());
        sleep(Duration::from_millis(500)).await;
        _ = self.intake.run(IntakeState::full_reverse());
        sleep(Duration::from_millis(1500)).await;
        _ = self.intake.run(IntakeState::full_brake());

        // Move to matchload thingy
        basic.drive_distance(&mut self.drivetrain, -6.0).await;

        let matchload_pos = Vec2::new(30.0, 0.0);
        basic.turn_to_point(&mut self.drivetrain, matchload_pos).await; // Turn towards goal
        let dist = (self.drivetrain.tracking.position() - matchload_pos).length();
        basic.drive_distance(&mut self.drivetrain, dist).await;

        // Engage matchload
        _ = self.trunk.set_state(TrunkState::Upper);
        basic.turn_to_heading(&mut self.drivetrain, Angle::from_degrees(270.0)).await;
        _ = self.drivetrain.model.drive_arcade(0.5, 0.0);
        sleep(Duration::from_secs(1)).await;

        // Take 3 team balls
        push_back::autons::intake_balls(&mut self.intake, Duration::from_secs(5)).await;
        _ = self.drivetrain.model.drive_arcade(0.0, 0.0);

        // Take 3 opp balls
        _ = self.intake.run(IntakeState::full_forward() - IntakeState::TRUNK);
        sleep(Duration::from_secs(2)).await;

        println!("Done");
    }

    async fn disabled(&mut self) {
        println!("Disabled!")
    }
}
