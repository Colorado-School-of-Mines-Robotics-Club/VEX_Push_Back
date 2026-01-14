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
    pub(super) mod basic {
        use core::time::Duration;

        use evian::{
            control::loops::{AngularPid, Pid}, math::Angle, motion::{Basic, Seeking}, prelude::Tolerances
        };

        pub const CONTROLLER: Basic<Pid, AngularPid> = Basic {
            linear_controller: LINEAR_PID,
            angular_controller: ANGULAR_PID,
            linear_tolerances: LINEAR_TOLERANCES,
            angular_tolerances: ANGULAR_TOLERANCES,
            timeout: Some(Duration::from_secs(5)),
        };

        pub const LINEAR_PID: Pid = Pid::new(0.035, 0.001, 0.02, None);
        pub const ANGULAR_PID: AngularPid = AngularPid::new(0.50, 0.02, 0.01, None);

        // TODO: actually check these
        pub const LINEAR_TOLERANCES: Tolerances = Tolerances::new()
            .error(0.5)
            .velocity(0.5)
            .duration(Duration::from_millis(15));
        pub const ANGULAR_TOLERANCES: Tolerances = Tolerances::new()
            .error(Angle::from_degrees(5.0).as_radians())
            .velocity(Angle::from_degrees(0.15).as_radians())
            .duration(Duration::from_millis(15));
    }

    // pub(super) mod seeking {
    //     use core::time::Duration;

    //     use evian::{
    //         control::loops::{AngularPid, Pid},
    //         motion::{Basic, Seeking},
    //         prelude::Tolerances,
    //     };

    //     pub const CONTROLLER: Seeking<Pid, Pid> = Seeking {
    //         linear_controller: LINEAR_PID,
    //         lateral_controller: LATERAL_PID,
    //         tolerances: LINEAR_TOLERANCES,
    //         timeout: Some(Duration::from_secs(5)),
    //     };

    //     pub const LINEAR_PID: Pid = Pid::new(0.035, 0.001, 0.02, None);
    //     pub const LATERAL_PID: Pid = Pid::new(0.055, 0.005, 0.00, None);

    //     pub const LINEAR_TOLERANCES: Tolerances = Tolerances::new()
    //         .error(0.5)
    //         .velocity(0.5)
    //         .duration(Duration::from_millis(15));
    // }
}

async fn intake_balls(intake: &mut IntakeSubsystem) {
    // Intake balls until both elevator top & bottom are broken (with debounce)
    _ = intake.run(IntakeState::full_forward() - IntakeState::TRUNK);
    let mut last_trigger = None;
    loop {
        last_trigger = if intake.sensors().contains(LineBreakState::ELEVATOR & LineBreakState::TRUNK) && last_trigger.is_none() {
            Some(Instant::now())
        } else { None };

        if let Some(last_trigger) = last_trigger && last_trigger.elapsed() > Duration::from_millis(100) {
            break;
        }

        sleep(Duration::from_millis(3)).await;
    }

    // Move balls until first ball hits outtake
    _ = intake.run(IntakeState::full_forward());
    while intake.sensors().contains(LineBreakState::OUTTAKE) {
        sleep(Duration::from_millis(3)).await;
    }
    _ = intake.run(IntakeState::full_brake());
}

// Note to self: trunk jam detection needs a higher limit?
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
    let mut basic = control::basic::CONTROLLER;
    // let mut seeking = control::seeking::CONTROLLER;

    println!("Calibrating...");
    if let Err(e) = copro.send_request(CalibrateRequest).await {
        eprintln!("Error calibrating: {e:?}");
        return;
    }

    // let starting_pos = drivetrain.tracking.position();
    // let starting_heading = drivetrain.tracking.heading();

    // sleep(Duration::from_millis(100)).await;

    // let start = drivetrain.tracking.forward_travel();
    // println!("START: {:.2}", start);

    // basic.drive_distance(drivetrain, 12.0).await;
    // dbg!(drivetrain.tracking.linear_velocity());

    // let end = drivetrain.tracking.forward_travel();
    // println!("END: {:.2}\nDIFF: {:.2}", end, end - start);

    println!("Turning to right!");

    let start = drivetrain.tracking.heading();
    println!("START: {:.2}", start.as_degrees());
    basic.turn_to_heading(drivetrain, Angle::from_degrees(0.0)).await;
    let end = drivetrain.tracking.heading();
    println!("END: {:.2}\nDIFF: {:.2}", end.as_degrees(), (((end - start).as_degrees() + 180.0) % 360.0) - 180.0);

    let mut i = 0;
    loop {
        let angle = if i % 2 == 0 {
            90.0
        } else { 0.0 };

        basic.turn_to_heading(drivetrain, Angle::from_degrees(angle)).await;

        i += 1;
        sleep(Duration::from_millis(500)).await;
    }

    // println!("Moving back...");
    // seeking.move_to_point(drivetrain, starting_pos).await;
    // basic.turn_to_heading(drivetrain, starting_heading).await;
    // println!("Done!");
}
