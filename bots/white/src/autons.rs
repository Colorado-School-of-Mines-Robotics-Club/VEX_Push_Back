use std::time::Duration;

use evian::{
	control::loops::AngularPid,
	drivetrain,
	math::{Angle, Vec2},
	motion::Basic,
	prelude::{Arcade, Tank, TracksForwardTravel, TracksHeading, TracksPosition},
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
pub async fn match_auton(robot: &mut Robot) {
	// Basic setup.
	_ = robot
		.pneumatics
		.extender
		.set_state(PneumaticState::Extended);

	let mut seeking = crate::control::SEEKING_CONTROLLER;

	let mut basic = crate::control::BASIC_CONTROLLER;

	// Robot drives to balls under the goal.

	basic.drive_distance(&mut robot.drivetrain, 20.0).await;

	seeking
		.move_to_point(&mut robot.drivetrain, Vec2::new(18.0, 53.5))
		.await;

	println!("{:?}", robot.drivetrain.tracking.position());

	basic
		.turn_to_heading(&mut robot.drivetrain, Angle::from_degrees(17.0))
		.await;

	basic
		.linear_controller
		.set_kp(basic.linear_controller.kp() * 3.0);
	basic.drive_distance(&mut robot.drivetrain, 3.0).await;
	basic
		.linear_controller
		.set_kp(basic.linear_controller.kp() / 3.0);

	_ = robot
		.pneumatics
		.front_bar
		.set_state(PneumaticState::Extended);

	println!("pneumatic!");
	sleep(Duration::from_secs(1)).await;

	// Robot intakes balls.
	_ = robot.drivetrain.model.drive_arcade(-0.25, -0.08);
	sleep(Duration::from_millis(250)).await;
	_ = robot.drivetrain.model.drive_arcade(0.10, -0.00);
	sleep(Duration::from_millis(150)).await;

	_ = robot.drivetrain.model.drive_arcade(-0.50, -0.17);
	sleep(Duration::from_millis(250)).await;
	_ = robot.drivetrain.model.drive_arcade(-0.50, 0.0);
	sleep(Duration::from_millis(1250)).await;

	robot.intake.run(IntakeState::full_forward());

	_ = robot.drivetrain.model.drive_arcade(0.15, 0.0);

	sleep(Duration::from_secs(3)).await;

	robot.intake.run(IntakeState::full_brake());

	_ = robot.drivetrain.model.drive_arcade(0.0, 0.0);

	// Robot positions itself towards middle bar and ejects balls.

	let target = Vec2::new(0.0, 46.7);
	basic.turn_to_point(&mut robot.drivetrain, target).await;
	seeking.move_to_point(&mut robot.drivetrain, target).await;

	// Robot positions itself near the tower and intakes the balls in the tower.

	// Robot turns 180 degress and positions itself toward the upper side beam, then ejects its balls.
}

// End of test code from Leo.

pub async fn pid_testing(robot: &mut Robot) {
	println!("Pid testing!!!");

	let mut basic = crate::control::BASIC_CONTROLLER;
	let mut seeking = crate::control::SEEKING_CONTROLLER;

	let start = robot.drivetrain.tracking.heading();

	let target = Angle::from_degrees(45.0);
	basic
		.turn_to_heading(&mut robot.drivetrain, target + start)
		.await;

	let end = robot.drivetrain.tracking.heading();

	println!(
		"---------------------------\nStart: {start:.03}\nEnd:   {end:.03}\nDiff:  {diff}\nError: {error}",
		start = start.as_degrees(),
		end = end.as_degrees(),
		diff = (end - start).as_degrees(),
		error = (target - (end - start)).as_degrees()
	)
}
