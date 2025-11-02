#![allow(dead_code)]
use core::time::Duration;

use evian::{
    math::Angle,
    motion::Basic,
    prelude::{TracksForwardTravel, TracksHeading, TracksPosition, TracksVelocity},
};
use shrewnit::{DegreesPerSecond, RadiansPerSecond};
use vexide::time::sleep;

use crate::robot::Robot;

mod control {
    use core::time::Duration;

    use evian::{
        control::loops::{AngularPid, Pid},
        prelude::Tolerances,
    };

    pub const PID: (Pid, AngularPid) = self::ABOT_PID;

    // TODO: actually check these
    pub const LINEAR_TOLERANCES: Tolerances = Tolerances::new()
        .error(0.25)
        .velocity(0.1)
        .duration(Duration::from_millis(15));
    pub const ANGULAR_TOLERANCES: Tolerances = Tolerances::new()
        .error(f64::to_radians(1.0))
        .velocity(0.05)
        .duration(Duration::from_millis(15));

    pub const DEVBOT_PID: (Pid, AngularPid) = (
        Pid::new(0.08, 0.0, 0.0, None),
        AngularPid::new(0.57, 0.00, 0.00, None),
    );
    pub const ABOT_PID: (Pid, AngularPid) = (
        Pid::new(0.0315, 0.0026, 0.0, None),
        AngularPid::new(0.0, 0.00, 0.00, None),
    );
}

pub async fn print_pose(robot: &mut Robot) {
    loop {
        println!(
            "LH: {:.4} LP: ({:.4}, {:.4}) VH: {:.4} VP: {:.4}",
            robot.drivetrain.tracking.heading().as_degrees(),
            robot.drivetrain.tracking.position().x,
            robot.drivetrain.tracking.position().y,
            (robot.drivetrain.tracking.angular_velocity() * RadiansPerSecond)
                .to::<DegreesPerSecond>(),
            robot.drivetrain.tracking.linear_velocity(),
        );
        sleep(Duration::from_millis(500)).await;
    }
}

pub async fn tune_pid(robot: &mut Robot) {
    let mut basic = Basic {
        linear_controller: control::PID.0,
        angular_controller: control::PID.1,
        linear_tolerances: control::LINEAR_TOLERANCES,
        angular_tolerances: control::ANGULAR_TOLERANCES,
        timeout: Some(Duration::from_secs(3)),
    };

    let start = robot.drivetrain.tracking.forward_travel();
    println!("START: {:.2}", start);

    basic.drive_distance(&mut robot.drivetrain, 12.0).await;

    let end = robot.drivetrain.tracking.forward_travel();
    println!("END: {:.2}\nDIFF: {:.2}", end, end - start);
}

pub async fn auton_1(robot: &mut Robot) {
    let mut basic = Basic {
        linear_controller: control::PID.0,
        angular_controller: control::PID.1,
        linear_tolerances: control::LINEAR_TOLERANCES,
        angular_tolerances: control::ANGULAR_TOLERANCES,
        timeout: None,
    };

    // The following coded is intended to make the robot drive forward, turn left, drive forward again,
    // grab a triball, drive backwards, turn left and drive forward.

    basic.drive_distance(&mut robot.drivetrain, 10.0).await;
    basic
        .turn_to_heading(&mut robot.drivetrain, Angle::from_degrees(90.0))
        .await;
    basic.drive_distance(&mut robot.drivetrain, 5.0).await;

    // _ = self.intake.set_velocity(200);
    sleep(Duration::from_secs(1)).await;
    // _ = self.intake.set_velocity(0);

    basic.drive_distance(&mut robot.drivetrain, 5.0).await;
    basic
        .turn_to_heading(&mut robot.drivetrain, Angle::from_degrees(180.0))
        .await;
}
