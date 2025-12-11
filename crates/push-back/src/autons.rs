#![allow(dead_code, unused_imports)]
use core::time::Duration;
use std::{future::join, time::Instant};

use coprocessor::requests::CalibrateRequest;
use evian::{
    drivetrain::model::Differential, math::Angle, prelude::{TracksForwardTravel, TracksHeading, TracksPosition, TracksVelocity}
};
use crate::subsystems::{
    copro::CoproSubsystem, drivetrain::DrivetrainSubsystem, intake::{IntakeState, IntakeSubsystem, LineBreakState}, trunk::{TrunkState, TrunkSubsystem}
};
use shrewnit::{DegreesPerSecond, RadiansPerSecond};
use vexide::time::sleep;

use crate::subsystems::copro::tracking::CoproTracking;

mod control {
    use core::time::Duration;

    use evian::{
        control::loops::{AngularPid, Pid},
        motion::Basic,
        prelude::Tolerances,
    };

    pub const BASIC_CONTROL: Basic<Pid, AngularPid> = Basic {
        linear_controller: LINEAR_PID,
        angular_controller: ANGULAR_PID,
        linear_tolerances: LINEAR_TOLERANCES,
        angular_tolerances: ANGULAR_TOLERANCES,
        timeout: Some(Duration::from_secs(5)),
    };

    pub const LINEAR_PID: Pid = Pid::new(0.035, 0.001, 0.02, None);
    pub const ANGULAR_PID: AngularPid = AngularPid::new(0.0, 0.00, 0.00, None);

    // TODO: actually check these
    pub const LINEAR_TOLERANCES: Tolerances = Tolerances::new()
        .error(0.5)
        .velocity(0.5)
        .duration(Duration::from_millis(15));
    pub const ANGULAR_TOLERANCES: Tolerances = Tolerances::new()
        .error(f64::to_radians(1.0))
        .velocity(0.05)
        .duration(Duration::from_millis(15));
}

// Note to self: trunk jam detection needs a higher limit
async fn move_ball_to(intake: &mut IntakeSubsystem, using: IntakeState, until: LineBreakState, move_from: Option<LineBreakState>) {
    _ = intake.run(using);
    let mut unjammed_from_at = Instant::now();
    while !intake.sensors().contains(until) {
        if intake.jams().intersects(using) {
            _ = intake.run(using.reverse());
            sleep(Duration::from_millis(100)).await;
            _ = intake.run(using);
            sleep(Duration::from_millis(100)).await;
        }
        if let Some(move_from) = move_from && unjammed_from_at.elapsed() > Duration::from_millis(750) && intake.sensors().contains(move_from) {
            _ = intake.run(using.reverse());
            sleep(Duration::from_millis(250)).await;
            _ = intake.run(using);
            unjammed_from_at = Instant::now()
        }
        sleep(Duration::from_millis(10)).await;
    }
}

pub async fn throw_balls(intake: &mut IntakeSubsystem) {
    loop {
        move_ball_to(
            intake,
            IntakeState::INTAKE_WHEELS,
            LineBreakState::INTAKE,
            None
        )
        .await;
        println!("Intake");

        move_ball_to(
            intake,
            IntakeState::INTAKE_WHEELS | IntakeState::BOTTOM | IntakeState::ELEVATOR_INTAKE,
            LineBreakState::ELEVATOR,
            None
        )
        .await;
        println!("Elevator intake");

        move_ball_to(
            intake,
            IntakeState::ELEVATOR_INTAKE | IntakeState::ELEVATOR,
            LineBreakState::TRUNK,
            None
        )
        .await;
        println!("Elevator");

        move_ball_to(
            intake,
            IntakeState::ELEVATOR | IntakeState::TRUNK,
            LineBreakState::OUTTAKE,
            Some(LineBreakState::TRUNK)
        )
        .await;
        println!("Thrown");
        sleep(Duration::from_millis(500)).await;

        // _ = robot.intake.run(IntakeState::full_brake());
    }
}

pub async fn print_state(intake: &mut IntakeSubsystem, trunk: &mut TrunkSubsystem) {
    let mut i = 0u8;
    _ = intake.run(IntakeState::FULL);
    _ = trunk.set_state(TrunkState::Down);
    loop {
        let current_sensors = intake.sensors();

        if i.is_multiple_of(50) {
            println!(
                "Line breaks: {}{}{}{} Jams: {:#05b}",
                if current_sensors.contains(LineBreakState::INTAKE) {
                    'I'
                } else {
                    'i'
                },
                if current_sensors.contains(LineBreakState::ELEVATOR) {
                    'E'
                } else {
                    'e'
                },
                if current_sensors.contains(LineBreakState::TRUNK) {
                    'T'
                } else {
                    't'
                },
                if current_sensors.contains(LineBreakState::OUTTAKE) {
                    'O'
                } else {
                    'o'
                },
                intake.jams().bits()
            );
        }

        i = i.wrapping_add(1);
        sleep(Duration::from_millis(2)).await;
    }
}

pub async fn print_pose(tracking: &CoproTracking) {
    loop {
        println!(
            "LH: {:.4} LP: ({:.4}, {:.4}) VH: {:.4} VP: {:.4}",
            tracking.heading().as_degrees(),
            tracking.position().x,
            tracking.position().y,
            (tracking.angular_velocity() * RadiansPerSecond)
                .to::<DegreesPerSecond>(),
            tracking.linear_velocity(),
        );
        sleep(Duration::from_millis(500)).await;
    }
}

pub async fn tune_pid(copro: &mut CoproSubsystem, drivetrain: &mut DrivetrainSubsystem<Differential, CoproTracking>) {
    let mut basic = control::BASIC_CONTROL;

    println!("Calibrating...");
    if let Err(e) = copro.port().send_request(CalibrateRequest).await {
        eprintln!("Error calibrating: {e:?}");
        return;
    }

    sleep(Duration::from_millis(100)).await;

    let start = drivetrain.tracking.forward_travel();
    println!("START: {:.2}", start);

    basic.drive_distance(drivetrain, 12.0).await;
    dbg!(drivetrain.tracking.linear_velocity());

    let end = drivetrain.tracking.forward_travel();
    println!("END: {:.2}\nDIFF: {:.2}", end, end - start);
}

// pub async fn auton_1(robot: &mut Robot) {
//     let mut basic = control::BASIC_CONTROL;

//     // The following coded is intended to make the robot drive forward, turn left, drive forward again,
//     // grab a triball, drive backwards, turn left and drive forward.

//     basic.drive_distance(&mut robot.drivetrain, 10.0).await;
//     basic
//         .turn_to_heading(&mut robot.drivetrain, Angle::from_degrees(90.0))
//         .await;
//     basic.drive_distance(&mut robot.drivetrain, 5.0).await;

//     // _ = self.intake.set_velocity(200);
//     sleep(Duration::from_secs(1)).await;
//     // _ = self.intake.set_velocity(0);

//     basic.drive_distance(&mut robot.drivetrain, 5.0).await;
//     basic
//         .turn_to_heading(&mut robot.drivetrain, Angle::from_degrees(180.0))
//         .await;
// }
