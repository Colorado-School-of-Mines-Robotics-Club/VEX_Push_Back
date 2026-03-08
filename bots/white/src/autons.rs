use std::time::{Duration, Instant};

use autons::{Selector, simple::Route};
use evian::{
	math::{Angle, Vec2},
	prelude::{Arcade, TracksHeading, TracksPosition},
};
use subsystems::{intake::IntakeState, pnemuatics::PneumaticState};
use vexide::{smart::motor::BrakeMode, time::sleep};

use crate::robot::Robot;

pub async fn do_nothing(_robot: &mut Robot) {
	println!("Doing absolutely nothing!!!");
}

pub async fn match_auton(robot: &mut Robot) {
	let matchload_line = 30.0;

	let start_time = Instant::now();
	// Basic setup.
	_ = robot
		.pneumatics
		.extender
		.set_state(PneumaticState::Extended);

	let mut basic = crate::control::BASIC_CONTROLLER;
	let mut _seeking = crate::control::SEEKING_CONTROLLER;

	// Line up with the center goal
	basic.drive_distance(&mut robot.drivetrain, 45.5).await;

	// Turn towards center goal
	basic
		.angular_controller
		.set_kp(basic.angular_controller.kp() * 1.5);
	basic
		.turn_to_heading(&mut robot.drivetrain, Angle::from_degrees(90.0 + 45.0))
		.await;
	basic
		.angular_controller
		.set_kp(basic.angular_controller.kp() / 1.5);

	// Align with center
	_ = robot.drivetrain.model.drive_arcade(0.25, 0.0);
	sleep(Duration::from_millis(1000)).await;
	_ = robot.drivetrain.model.drive_arcade(-0.15, 0.0);
	sleep(Duration::from_millis(350)).await;
	_ = robot.drivetrain.model.drive_arcade(0.0, 0.0);

	// Outtake preload ball into center goal
	robot.intake.run(IntakeState {
		top: -1.0,
		middle: -1.0,
		bottom: -0.50,
	});
	sleep(Duration::from_secs(1)).await;
	robot.intake.run(IntakeState::full_brake());

	// Go generally towards aligned with the matchload
	basic
		.drive_distance_at_heading(
			&mut robot.drivetrain,
			-25.0,
			Angle::from_degrees(-45.0 + 180.0),
		)
		.await;

	// Move to matchload line
	let dist_x = (robot.drivetrain.tracking.position().x - matchload_line).abs();
	let dist = (dist_x / (robot.drivetrain.tracking.heading()).cos()).abs();

	basic.drive_distance(&mut robot.drivetrain, -dist).await;

	// Turn towards matchload
	basic
		.angular_controller
		.set_kp(basic.angular_controller.kp() * 1.1);
	basic
		.turn_to_heading(&mut robot.drivetrain, Angle::from_degrees(-90.0))
		.await;
	basic
		.angular_controller
		.set_kp(basic.angular_controller.kp() / 1.1);

	// Back up as a precaution for the front bar
	_ = robot.drivetrain.model.drive_arcade(-0.17, 0.0);
	sleep(Duration::from_millis(350)).await;

	// Intake from matchload
	_ = robot
		.pneumatics
		.front_bar
		.set_state(PneumaticState::Extended);
	_ = robot
		.pneumatics
		.outtake_adjuster
		.set_state(PneumaticState::Extended);
	_ = robot.pneumatics.flap.set_state(PneumaticState::Contracted);
	sleep(Duration::from_millis(500)).await;
	_ = robot.drivetrain.model.drive_arcade(0.35, 0.0);
	robot.intake.run(IntakeState::full_forward());
	sleep(Duration::from_millis(2500)).await;
	_ = robot.drivetrain.model.drive_arcade(0.0, 0.0);
	sleep(Duration::from_secs(2)).await;

	// Move to long goal position
	robot.intake.run(IntakeState::full_brake());

	_ = robot.drivetrain.model.drive_arcade(-0.35, 0.01);
	sleep(Duration::from_secs(2)).await;
	_ = robot.drivetrain.model.drive_arcade(0.0, 0.0);

	// Eject 3 balls into long goal
	_ = robot
		.pneumatics
		.outtake_adjuster
		.set_state(PneumaticState::Extended);
	_ = robot.pneumatics.flap.set_state(PneumaticState::Extended);
	robot.intake.run(IntakeState {
		top: 1.0,
		middle: 0.0,
		bottom: 0.0,
	});
	sleep(Duration::from_secs(2)).await;
	robot.intake.run(IntakeState::full_brake());

	// Back up and eject balls out of the intake
	basic.drive_distance(&mut robot.drivetrain, 12.0).await;
	_ = robot
		.pneumatics
		.outtake_adjuster
		.set_state(PneumaticState::Contracted);
	robot.intake.run(IntakeState::full_forward());
	sleep(Duration::from_secs(2)).await;

	while let error =
		(robot.drivetrain.tracking.heading().wrapped_full() - Angle::from_degrees(90.0 + 180.0))
		&& error.abs() > Angle::from_degrees(1.0)
	{
		_ = robot
			.drivetrain
			.model
			.drive_arcade(0.0, 0.12 * error.signum());
		sleep(Duration::from_millis(5)).await;
	}
	robot.drivetrain.brake(BrakeMode::Hold);

	// Go back to matchload and grab balls
	_ = robot
		.pneumatics
		.outtake_adjuster
		.set_state(PneumaticState::Extended);
	_ = robot.pneumatics.flap.set_state(PneumaticState::Contracted);
	_ = robot.drivetrain.model.drive_arcade(0.35, 0.0);
	sleep(Duration::from_secs(2)).await;

	// Score the balls on long goal
	basic.drive_distance(&mut robot.drivetrain, -36.0).await;
	_ = robot.drivetrain.model.drive_arcade(-0.35, 0.0);
	sleep(Duration::from_millis(500)).await;

	_ = robot.pneumatics.flap.set_state(PneumaticState::Extended);
	robot.intake.run(IntakeState::full_forward());
	sleep(Duration::from_secs(2)).await;

	return;

	// Move back a bit and eject all balls (wrong color)
	_ = robot.drivetrain.model.drive_arcade(0.40, 0.0);
	sleep(Duration::from_millis(500)).await;
	_ = robot.drivetrain.model.drive_arcade(0.0, 0.0);

	_ = robot
		.pneumatics
		.front_bar
		.set_state(PneumaticState::Contracted);
	basic
		.angular_controller
		.set_kp(basic.angular_controller.kp() * 3.05);
	basic
		.turn_to_heading(&mut robot.drivetrain, Angle::from_degrees(75.0 + 180.0))
		.await;
	robot.intake.run(IntakeState::full_forward());
	sleep(Duration::from_millis(1500)).await;

	let dist_x = (robot.drivetrain.tracking.position().x - matchload_line).abs();
	let dist = (dist_x / (robot.drivetrain.tracking.heading()).cos()).abs();

	basic.drive_distance(&mut robot.drivetrain, dist).await;

	basic
		.turn_to_heading(&mut robot.drivetrain, Angle::from_degrees(90.0 + 180.0))
		.await;
	basic
		.angular_controller
		.set_kp(basic.angular_controller.kp() / 3.05);

	if robot.drivetrain.tracking.position().y() >= 15.0 {
		// Back up as a precaution for the front bar
		_ = robot.drivetrain.model.drive_arcade(-0.17, 0.0);
		sleep(Duration::from_millis(350)).await;
	}

	_ = robot
		.pneumatics
		.front_bar
		.set_state(PneumaticState::Extended);

	// Move the rest of the distance and intake balls
	_ = robot.pneumatics.flap.set_state(PneumaticState::Contracted);
	_ = robot.drivetrain.model.drive_arcade(0.35, 0.0);
	sleep(Duration::from_secs(3)).await;

	// Drive to long goal
	basic.timeout = Some(Duration::from_secs(1));
	basic
		.drive_distance_at_heading(
			&mut robot.drivetrain,
			-40.0,
			Angle::from_degrees(180.0 + 90.0),
		)
		.await;
	basic.timeout = crate::control::BASIC_CONTROLLER.timeout;

	_ = robot
		.pneumatics
		.front_bar
		.set_state(PneumaticState::Contracted);

	// Outtake the balls!!!!!!!!!
	_ = robot.pneumatics.flap.set_state(PneumaticState::Extended);
	sleep(Duration::from_secs(2)).await;
	robot.intake.run(IntakeState::full_brake());

	// Move back from goal
	_ = robot.drivetrain.model.drive_arcade(0.30, 0.0);
	sleep(Duration::from_millis(250)).await;
	_ = robot.drivetrain.model.drive_arcade(0.0, 0.0);
	sleep(Duration::from_millis(250)).await;

	// Go to park zone
	// basic
	// 	.angular_controller
	// 	.set_kp(basic.angular_controller.kp() * 2.3);
	// basic
	// 	.turn_to_heading(&mut robot.drivetrain, Angle::from_degrees(180.0 + 45.0))
	// 	.await;
	// basic
	// 	.angular_controller
	// 	.set_kp(basic.angular_controller.kp() / 2.3);

	println!("Done: {}s", start_time.elapsed().as_secs_f32());
}

pub async fn pid_testing(robot: &mut Robot) {
	println!("Pid testing!!!");

	let mut _basic = crate::control::BASIC_CONTROLLER;
	let mut seeking = crate::control::SEEKING_CONTROLLER;
	// seeking.linear_controller = Pid::new(0.0, 0.0, 0.0, None);

	let start = robot.drivetrain.tracking.heading();

	let target = Angle::from_degrees(-45.0);
	seeking
		.move_to_point(&mut robot.drivetrain, Vec2::new(24.0, 24.0))
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

pub async fn skills_auton(robot: &mut Robot) {
	robot.intake.run(IntakeState::full_forward());
	_ = robot.drivetrain.model.drive_arcade(0.4, 0.0);
	sleep(Duration::from_millis(1250)).await;
	_ = robot.drivetrain.model.drive_arcade(0.0, 0.0);
	sleep(Duration::from_secs(3)).await;
}

#[allow(dead_code)]
pub struct StubSelector<R> {
	route: Option<Route<R>>,
}

#[allow(dead_code)]
impl<R> StubSelector<R> {
	pub fn new<const N: usize>(selected: &'static str, routes: [Route<R>; N]) -> Self {
		Self {
			route: routes.into_iter().find(|r| r.name == selected),
		}
	}
}

impl<R> Selector<R> for StubSelector<R> {
	async fn run(&self, robot: &mut R) {
		if let Some(route) = &self.route {
			(route.callback)(robot).await;
		}
	}
}
