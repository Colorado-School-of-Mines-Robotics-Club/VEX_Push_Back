use std::{time::Duration};

use evian::{
	control::loops::AngularPid, drivetrain, math::{Angle, Vec2}, motion::Basic, prelude::{Tank, TracksForwardTravel, TracksHeading, TracksPosition}
};
use shrewnit::Seconds;
use slintui::{OdometryPageState, Pose, plotting::TrackedPid, slint::ComponentHandle as _};
use subsystems::{intake::IntakeState, pnemuatics::PneumaticState};
use vexide::time::sleep;

use crate::robot::Robot;

pub async fn do_nothing(_robot: &mut Robot) {
	println!("Doing absolutely nothing!!!");
}

// Start of Leo's test code.
pub async fn testAuton(robot: &mut Robot){
	// Basic setup.
	robot.pneumatics.extender.set_state(PneumaticState::Extended);

	let mut seeking = crate::control::SEEKING_CONTROLLER;

	let mut basic = crate::control::BASIC_CONTROLLER;

	// Robot drives to balls under the bar.

	seeking.move_to_point(&mut robot.drivetrain, Vec2::new(0.0, 20.0)).await;
	
	seeking.move_to_point(&mut robot.drivetrain, Vec2::new(15.0, 57.0)).await;

	basic.turn_to_heading(&mut robot.drivetrain, Angle::from_degrees(5.0)).await;

	basic.drive_distance(&mut robot.drivetrain, 4.0).await;

	robot.pneumatics.front_bar.set_state(PneumaticState::Extended);

	sleep(Duration::from_secs(1)).await;

	// Robot intakes balls.

	basic.drive_distance(&mut robot.drivetrain, -12.0).await;

	robot.intake.run(IntakeState::full_forward());

	robot.drivetrain.model.drive_tank(0.1, 0.15);

	sleep(Duration::from_secs(2)).await;

	robot.intake.run(IntakeState::full_brake());

	robot.drivetrain.model.drive_tank(0.0, 0.0);

	// Robot positions itself towards middle bar and ejects balls.

	seeking.move_to_point(&mut robot.drivetrain, Vec2::new(0.0, 30.0)).await;

	// Robot positions itself near the tower and intakes the balls in the tower.

	// Robot turns 180 degress and positions itself toward the upper side beam, then ejects its balls.


	
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
