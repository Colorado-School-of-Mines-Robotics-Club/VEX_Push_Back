use crate::robot::Robot;
use evian::{
	math::{Angle, Vec2},
	prelude::{TracksForwardTravel, TracksHeading, TracksPosition},
};

pub async fn do_nothing(_robot: &mut Robot) {
	println!("Doing absolutely nothing!!!");
}

pub async fn pid_testing(robot: &mut Robot) {
	println!("Pid testing!!!");

	let mut seeking = crate::control::SEEKING_CONTROLLER;

	let target = Vec2::new(12.0, 24.0);
	let start = robot.drivetrain.tracking.position();

	seeking
		.move_to_point(&mut robot.drivetrain, Vec2::new(12.0, 24.0))
		.await;

	let end = robot.drivetrain.tracking.position();

	println!(
		"---------------------------\nStart: {start:.03}\nEnd:   {end:.03}\nDiff:  {diff}\nError: {error}",
		diff = end - start,
		error = target - (end - start)
	)
}
