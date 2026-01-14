use std::time::{Duration, Instant};

use coprocessor::requests::CalibrateRequest;
use evian::{control::loops::{AngularPid, Pid}, motion::Basic, prelude::{Arcade, Tank, Tolerances}};
use push_back::subsystems::{intake::IntakeState, trunk::TrunkState, ControllableSubsystem};
use vexide::prelude::*;

use crate::robot::Robot;

async fn bad_auton(robot: &mut Robot) {
    let velocity = 1.0;
    let start = Instant::now();
    let time = Duration::from_millis(1400);
    while let elapsed = start.elapsed() && elapsed < time {
        let coeff = (1.0 - elapsed.div_duration_f64(time));
        _ = robot.drivetrain.model.drive_arcade(velocity * coeff, -0.15 * coeff);
        sleep(Duration::from_millis(3)).await;
    }

    _ = robot.drivetrain.model.drive_arcade(0.0, 0.05);
    sleep(Duration::from_millis(450)).await;
    _ = robot.drivetrain.model.drive_arcade(0.0, 0.0);

    _ = robot.trunk.set_state(TrunkState::Upper);

    let velocity = 1.0;
    let start = Instant::now();
    let time = Duration::from_millis(1250);
    while let elapsed = start.elapsed() && elapsed < time {
        _ = robot.drivetrain.model.drive_arcade(velocity * (1.0 - elapsed.div_duration_f64(time)), 0.0);
        sleep(Duration::from_millis(3)).await;
    }

    _ = robot.drivetrain.model.drive_arcade(0.0, 0.0);
    _ = robot.intake.run(IntakeState::FULL - IntakeState::TRUNK);
    sleep(Duration::from_secs(3)).await;
    _ = robot.intake.run(IntakeState::empty());

    let velocity = -0.5;
    let start = Instant::now();
    let time = Duration::from_millis(1000);
    while let elapsed = start.elapsed() && elapsed < time {
        _ = robot.drivetrain.model.drive_arcade(velocity * (1.0 - elapsed.div_duration_f64(time)), 0.01);
        sleep(Duration::from_millis(3)).await;
    }
    _ = robot.drivetrain.model.drive_arcade(0.0, 0.0);


    let velocity = 0.05;
    let start = Instant::now();
    let time = Duration::from_millis(750);
    while let elapsed = start.elapsed() && elapsed < time {
        _ = robot.drivetrain.model.drive_arcade(0.1, velocity * (1.0 - (time.as_millis_f64() / 2.0 - elapsed.as_millis_f64()).abs()));
        sleep(Duration::from_millis(3)).await;
    }
    _ = robot.drivetrain.model.drive_arcade(0.0, 0.0);
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
        _ = self.drivetrain.drivetrain.model.drive_tank(0.0, 0.0);
        println!("Auton!");

        // if let Ok(file) = File::options().read(true).open("record30.txt") {
        //     let mut subsystems = HashMap::<&'static str, &mut dyn ControllableSubsystem>::new();
        //     subsystems.insert("drivetrain", &mut self.drivetrain);
        //     subsystems.insert("intake", &mut self.intake);
        //     subsystems.insert("trunk", &mut self.trunk);
        //     self.replay.replay(file, subsystems).await;
        // }

        // push_back::autons::print_pose(self).await;
        // push_back::autons::auton_1(self).await;
        push_back::autons::tune_pid(&mut self.coprocessor, &mut self.drivetrain).await;
        // push_back::autons::print_state(&mut self.intake, &mut self.trunk).await;
        // push_back::autons::throw_balls(self).await;
        // push_back::autons::print_pose(&self.drivetrain.tracking).await;
        // bad_auton(self).await;

    }

    async fn disabled(&mut self) {
        println!("Disabled!")
    }
}
