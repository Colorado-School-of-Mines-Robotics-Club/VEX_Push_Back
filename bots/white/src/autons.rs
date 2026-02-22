use evian::{
	control::loops::AngularPid, drivetrain, math::{Angle, Vec2}, motion::Basic, prelude::{TracksForwardTravel, TracksHeading, TracksPosition}
};
use slintui::{OdometryPageState, Pose, plotting::TrackedPid, slint::ComponentHandle as _};
use subsystems::{intake::IntakeState, pnemuatics::PneumaticState};
use vexide::time::sleep;

use crate::robot::Robot;

pub async fn do_nothing(_robot: &mut Robot) {
	println!("Doing absolutely nothing!!!");
}

// Start of Leo's test code.
pub async fn testAuton(robot: &mut Robot){
	robot.pneumatics.extender.set_state(PneumaticState::Extended);

	
	
}

// End of test code from Leo.

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
