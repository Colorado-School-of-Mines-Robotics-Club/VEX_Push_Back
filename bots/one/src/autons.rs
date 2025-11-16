#![allow(dead_code)]
use core::time::Duration;

use evian::{
    math::Angle,
    prelude::{TracksForwardTravel, TracksHeading, TracksPosition, TracksVelocity},
};
use shrewnit::{DegreesPerSecond, RadiansPerSecond};
use vexide::time::sleep;

use crate::robot::Robot;

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
        timeout: None,
    };

    pub const LINEAR_PID: Pid = Pid::new(0.00, 0.00, 0.0, None);
    pub const ANGULAR_PID: AngularPid = AngularPid::new(0.0, 0.00, 0.00, None);

    // TODO: actually check these
    pub const LINEAR_TOLERANCES: Tolerances = Tolerances::new()
        .error(0.25)
        .velocity(0.1)
        .duration(Duration::from_millis(15));
    pub const ANGULAR_TOLERANCES: Tolerances = Tolerances::new()
        .error(f64::to_radians(1.0))
        .velocity(0.05)
        .duration(Duration::from_millis(15));
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
    let mut basic = control::BASIC_CONTROL;

    let start = robot.drivetrain.tracking.forward_travel();
    println!("START: {:.2}", start);

    basic.drive_distance(&mut robot.drivetrain, 12.0).await;

    let end = robot.drivetrain.tracking.forward_travel();
    println!("END: {:.2}\nDIFF: {:.2}", end, end - start);
}

pub async fn auton_1(robot: &mut Robot) {
    let mut basic = control::BASIC_CONTROL;

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
