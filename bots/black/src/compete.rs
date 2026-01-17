use std::time::{Duration, Instant};

use coprocessor::requests::CalibrateRequest;
use evian::{control::loops::{AngularPid, Pid}, math::Angle, motion::Basic, prelude::*};
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
        _ = self.coprocessor.send_request(CalibrateRequest).await;
        let mut basic: Basic<Pid, AngularPid> = crate::control::basic::CONTROLLER;
        basic.timeout = Some(Duration::from_secs(1));
        basic.turn_to_heading(&mut self.drivetrain, (Angle::from_degrees(90.0+40.0)).wrapped_full()).await;
        basic.timeout = Some(Duration::from_secs(3));
        basic.drive_distance(&mut self.drivetrain, 42.0).await;
        basic.timeout = Some(Duration::from_secs(1));
        basic.turn_to_heading(&mut self.drivetrain, (Angle::from_degrees(90.0+70.0)).wrapped_full()).await;
        _ = self.intake.run(IntakeState::full_forward() - IntakeState::TRUNK - IntakeState::ELEVATOR);
        sleep(Duration::from_millis(5)).await;
        basic.drive_distance(&mut self.drivetrain, 7.0).await;
        _ = self.intake.run(IntakeState::full_brake());
        basic.drive_distance(&mut self.drivetrain, -7.0).await;
        basic.turn_to_heading(&mut self.drivetrain, (Angle::from_degrees(90.0+360.0 - 90.0)).wrapped_full()).await;
        basic.timeout = Some(Duration::from_secs(3));
        basic.drive_distance(&mut self.drivetrain, 23.0).await;
        basic.timeout = Some(Duration::from_secs(1));
        basic.turn_to_heading(&mut self.drivetrain, (Angle::from_degrees(90.0+360.0 - 45.0)).wrapped_full()).await;
        basic.timeout = Some(Duration::from_millis(200));
        basic.drive_distance(&mut self.drivetrain, 500.0).await;
        _ = self.intake.run(IntakeState::full_forward());
        sleep(Duration::from_secs(3)).await;
        _ = self.intake.run(IntakeState::full_brake());
        basic.timeout = Some(Duration::from_secs(1));
        basic.turn_to_heading(&mut self.drivetrain, (Angle::from_degrees(90.0+360.0 - 45.0 - 180.0)).wrapped_full()).await;
        basic.timeout = Some(Duration::from_secs(3));
        _ = self.trunk.set_state(TrunkState::Upper);
        basic.drive_distance_at_heading(&mut self.drivetrain, 45.0, (Angle::from_degrees(90.0+360.0 - 45.0 - 170.0)).wrapped_full()).await;
        basic.timeout = Some(Duration::from_secs(1));
        basic.turn_to_heading(&mut self.drivetrain, (Angle::from_degrees(-85.0)).wrapped_full()).await;
        basic.timeout = Some(Duration::from_millis(450));
        _ = self.intake.run(IntakeState::full_forward() - IntakeState::TRUNK);
        basic.drive_distance(&mut self.drivetrain, 500.0).await;
        sleep(Duration::from_secs(3)).await;
        _ = self.intake.run(IntakeState::full_brake());
        basic.timeout = Some(Duration::from_secs(1));
        basic.drive_distance(&mut self.drivetrain, -10.0).await;
        _ = self.trunk.set_state(TrunkState::Lower);
        _ = self.intake.run(IntakeState::full_reverse());
        sleep(Duration::from_millis(200)).await;
        _ = self.intake.run(IntakeState::full_brake());
        basic.turn_to_heading(&mut self.drivetrain, (Angle::from_degrees(85.5)).wrapped_full()).await;
        basic.drive_distance(&mut self.drivetrain, 30.0).await;
        _ = self.intake.run(IntakeState::full_forward());
        sleep(Duration::from_secs(3)).await;
        _ = self.intake.run(IntakeState::full_reverse());
        sleep(Duration::from_millis(150)).await;
        _ = self.intake.run(IntakeState::full_forward());
        sleep(Duration::from_secs(3)).await;
        _ = self.intake.run(IntakeState::full_reverse());
        sleep(Duration::from_millis(150)).await;
        _ = self.intake.run(IntakeState::full_forward());
        sleep(Duration::from_secs(3)).await;
        _ = self.intake.run(IntakeState::full_reverse());
        sleep(Duration::from_millis(150)).await;
        _ = self.intake.run(IntakeState::full_forward());
        sleep(Duration::from_secs(3)).await;
        _ = self.intake.run(IntakeState::full_reverse());
        sleep(Duration::from_millis(150)).await;

    }

    async fn disabled(&mut self) {
        println!("Disabled!")
    }
}
