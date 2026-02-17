use std::time::Duration;

use evian::{
	control::loops::AngularPid,
	math::Angle,
	motion::Basic,
	prelude::{TracksForwardTravel, TracksHeading, TracksPosition},
};
use slintui::{OdometryPageState, Pose, plotting::TrackedPid, slint::ComponentHandle as _};
use vexide::time::sleep;

use crate::robot::Robot;

pub async fn do_nothing(_robot: &mut Robot) {
	println!("Doing absolutely nothing!!!");
}

// tmp pls fix at some point me thanks
pub async fn show_pose(robot: &mut Robot) {
	loop {
		let position = robot.drivetrain.tracking.position();
		robot
			.ui
			.app()
			.global::<OdometryPageState>()
			.set_position(Pose {
				x: position.x as f32,
				y: position.y as f32,
				h: robot.drivetrain.tracking.heading().as_degrees() as f32,
			});
		sleep(Duration::from_millis(500)).await;

		dbg!(robot.drivetrain.tracking.forward_travel());
	}
}

pub async fn pid_testing(robot: &mut Robot) {
	println!("Pid testing!!!");

	let mut basic = Basic {
		// linear_controller: crate::control::basic::LINEAR_PID,
		linear_controller: TrackedPid::new(crate::control::basic::LINEAR_PID, robot.ui.app()),
		angular_controller: AngularPid::new(0.0, 0.0, 0.0, None),
		linear_tolerances: crate::control::basic::LINEAR_TOLERANCES,
		angular_tolerances: crate::control::basic::ANGULAR_TOLERANCES,
		timeout: Some(Duration::from_secs(5)),
	};

	let dist = 12.0 * 4.0;
	let start = robot.drivetrain.tracking.forward_travel();
	basic.drive_distance(&mut robot.drivetrain, dist).await;
	let end = robot.drivetrain.tracking.forward_travel();

	println!(
		"---------------------------\nStart: {start:.03}\nEnd:   {end:.03}\nDiff:  {diff}\nError: {error}",
		diff = end - start,
		error = dist - (end - start)
	)
}

pub async fn pid_angle_testing(robot: &mut Robot) {
	println!("Pid rotation testing!!!");

	let mut basic = crate::control::basic::CONTROLLER;

	let dist = Angle::from_degrees(90.0);
	let start = robot.drivetrain.tracking.heading();
	basic
		.turn_to_heading(&mut robot.drivetrain, start + dist)
		.await;
	let end = robot.drivetrain.tracking.heading();

	println!(
		"---------------------------\nStart: {start:.03}\nEnd:   {end:.03}\nDiff:  {diff}\nError: {error}",
		start = start.as_degrees(),
		end = end.as_degrees(),
		diff = (end - start).wrapped_full().as_degrees(),
		error = (dist - (end - start)).wrapped_full().as_degrees()
	)
}
