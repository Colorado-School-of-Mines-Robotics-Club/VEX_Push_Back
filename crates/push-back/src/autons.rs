#![allow(dead_code, unused_imports)]
use core::time::Duration;
use std::{future::join, time::Instant};

use coprocessor::requests::CalibrateRequest;
use evian::{
    control::loops::{AngularPid, Pid}, drivetrain::model::Differential, math::Angle, motion::Basic, prelude::{TracksForwardTravel, TracksHeading, TracksPosition, TracksVelocity}
};
use crate::subsystems::{
    copro::CoproSubsystem, drivetrain::DrivetrainSubsystem, intake::{IntakeState, IntakeSubsystem, LineBreakState}, trunk::{TrunkState, TrunkSubsystem}
};
use shrewnit::{DegreesPerSecond, RadiansPerSecond};
use vexide::{controller::ControllerState, prelude::Controller, time::sleep};

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

pub async fn intake_balls(intake: &mut IntakeSubsystem, timeout: Duration) {
    let start = Instant::now();
    // Intake balls until both elevator top & bottom are broken (with debounce)
    _ = intake.run(IntakeState::full_forward() - IntakeState::TRUNK);
    let mut last_trigger = None;
    loop {
        if start.elapsed() >= timeout {
            break;
        }

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
    while intake.sensors().contains(LineBreakState::OUTTAKE) && start.elapsed() < timeout {
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

pub async fn print_pose(tracking: &CoproTracking) -> ! {
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

async fn wait_calibration(copro: &mut CoproSubsystem) {
    println!("Calibrating...");
    while copro.send_request(CalibrateRequest).await.is_err() {
        eprintln!("Failed calibration...");
        sleep(Duration::from_secs(1)).await;
    }
    sleep(Duration::from_secs(1)).await;
}

pub async fn tune_basic_pid(basic: &mut Basic<Pid, AngularPid>, controller: &Controller, copro: &mut CoproSubsystem, drivetrain: &mut DrivetrainSubsystem<Differential, CoproTracking>) {
    'outer: loop {
        println!("--- L1: Fwd 1ft, L2: Fwd 3ft, L3, R1: +15deg, R2: +90deg ---");

        loop {
            let Ok(state) = controller.state() else {
                println!("Controller not connected :(");
                return;
            };

            if state.button_x.is_now_pressed() {
                break 'outer;
            }

            if state.button_l1.is_now_pressed() {
                wait_calibration(copro).await;

                let start = drivetrain.tracking.forward_travel();
                let dist = 12.0 * if state.button_b.is_pressed() { -1.0 } else { 1.0 };
                println!("START: {:.2}", start);

                basic.drive_distance(drivetrain, dist).await;

                let end = drivetrain.tracking.forward_travel();
                println!("END: {:.2}\nDIFF: {:.2}\nERR: {:.2}", end, end - start, end - start - dist);
            } else if state.button_l2.is_now_pressed() {
                wait_calibration(copro).await;

                let start = drivetrain.tracking.forward_travel();
                let dist = 36.0 * if state.button_b.is_pressed() { -1.0 } else { 1.0 };
                println!("START: {:.2}", start);

                basic.drive_distance(drivetrain, dist).await;

                let end = drivetrain.tracking.forward_travel();
                println!("END: {:.2}\nDIFF: {:.2}\nERR: {:.2}", end, end - start, (end - start - dist) * dist.signum());
            } else if state.button_r1.is_now_pressed() {
                wait_calibration(copro).await;

                let start = drivetrain.tracking.heading();
                let dist = Angle::from_degrees(15.0) * if state.button_b.is_pressed() { -1.0 } else { 1.0 };
                println!("START: {:.2}", start.as_degrees());

                basic.turn_to_heading(drivetrain, (start + dist).wrapped_full()).await;

                let end = drivetrain.tracking.heading();
                println!("END: {:.2}\nDIFF: {:.2}\nERR: {:.2}", end.as_degrees(), (end - start).as_degrees(), (end - start - dist).as_degrees());
            } else if state.button_r2.is_now_pressed() {
                wait_calibration(copro).await;

                let start = drivetrain.tracking.heading();
                let dist = Angle::from_degrees(90.0) * if state.button_b.is_pressed() { -1.0 } else { 1.0 };
                println!("START: {:.2}", start.as_degrees());

                basic.turn_to_heading(drivetrain, (start + dist).wrapped_full()).await;

                let end = drivetrain.tracking.heading();
                println!("END: {:.2}\nDIFF: {:.2}\nERR: {:.2}", end.as_degrees(), (end - start).as_degrees(), (end - start - dist).as_degrees());
            } else {
                sleep(Duration::from_millis(5)).await;
                continue
            }

            break;
        }

        sleep(Duration::from_millis(500)).await;
    }

    // let start = drivetrain.tracking.heading();
    // println!("START: {:.2}", start.as_degrees());
    // basic.turn_to_heading(drivetrain, Angle::from_degrees(0.0)).await;
    // let end = drivetrain.tracking.heading();
    // println!("END: {:.2}\nDIFF: {:.2}", end.as_degrees(), (((end - start).as_degrees() + 180.0) % 360.0) - 180.0);
}
